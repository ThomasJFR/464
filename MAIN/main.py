# MECH 464 Group 1
# This file is the primary file for running the 
# Pick and place operation. It contains the main while loop
# and the system state machine.

# IMPORTS
import numpy as np
import threading
import dvrk
import PyKDL
from time import time
from collections import deque
from enum import Enum
from moves import 
from utils import FakeWaiter, PSMSequence, dist

items = deque()
bowls = list()
Z_OFFSET = 0.05

def capture_system():
    "Function that gets a photo of the system that we can extract features from"
    pass

def extract_system_features(image):
    """
    Extract targets and bowls from an image
    """
    # Process image to identify positions of targets and bowls

    # Dummy logic:
    features = dict()
    #psm1pos = np.array([-1.513,  -0.0819994, 05]) 
    features["items"] = [
       # np.array([ -1.523,  -0.0469994,      0.5655])
    #]
        np.array([-1.527791262, 0.05018193647, 0.674774766]),
        np.array([-1.516814113, -0.04662011936, 0.6747748256]),
        np.array([-1.547450662, -0.05359531567, 0.674774766]),
        np.array([-1.536615372, 0.003250310896, 0.674774766]),
        np.array([-1.51279664, 0.02789815143, 0.674774766]),
        np.array([-1.550005555, -0.02500112727, 0.6747747064]),
    ]

    for i in range(len(features["items"])):
        features["items"][i][2] = 0.5655
    #features["items"] = [x -  psm1pos for x in features["items"]]
    features["bowls"] = [
       # np.array([-1.443, -0.0020, 0.5655]),
        np.array([-1.473, -0.0570, 0.5655]) #- psm1pos,
    ]
    return features

def dispatch_item(p):
    """
    This function  interfaces with the computer vision part of the system and decides
    which object to try to pick up next based on which arm is asking, and whether it has
    the mutex.
    """
    global items
    if len(items) == 0:
        return None, True
    # Get the closest target
    item = items.popleft() if (p.name() == "PSM1") else items.pop()
    return item, False

def dispatch_bowl(p):
    global bowls
    if len(bowls) == 0:
        return None, True
    bowl = bowls[0] if (p.name() == "PSM1") else bowls[-1]  # bowls[0] is closest to psm1; bowls[-1] to psm2
    return bowl, False


def initialize(armName):
    """
    This is a function to initialize and home each arm, and to send each arm to the 
    mainloop once initialized.
    """
    # Initialize arm
    p = dvrk.psm(armName)

    # Home the arm
    p.enable()
    p.home()
    return p

class PSMState:
    s = Enum("PSM States", [
        "Standby",
        "RequestItem",
        "MoveToItem", 
        "Descend",
        "Grab",
        "Ascend",
        "RequestBowl",
        "MoveToBowl",
        "Release",
        "Home",
        "Finished"
    ])
    
    def __init__(self):
        self.s = PSMState.s.Standby 
        self.s_next = PSMState.s.Standby
        self.target = None
        self.waiter = FakeWaiter()
        self.has_lock = False

def collision_risk(state1, state2):
    target1 = state1.target
    target2 = state2.target
    if target1 is None or target2 is None:
        return [False]

    conditions = [
        any([target1[1] < target2[1],
        dist(target1, target2, xy=True) < 0.05]),
        ("Move" in state1.s.name or "Move" in state2.s.name)

    ]
    #print(conditions)
    return conditions
    
    return all([
        #dist(target1[0:1], target2[0:1]) < 0.02,  # X-Y distance too close
        target1[1] > target2[1]  # Y boundary crossing
    ])

def mainloop(psm1, psm2):
    # State variables
    psm1_state, psm2_state = PSMState(), PSMState() 
    psm1_active, psm2_active = True,  True
    # TEMP
    start = time()
    lastprint = 0
    while psm1_active or psm2_active:
        now = time()
        psm1_active = psm1_state.s is not PSMState.s.Finished
        psm2_active = psm2_state.s is not PSMState.s.Finished

        if psm1_active:
            will_collide = all(collision_risk(psm1_state, psm2_state)) 
            psm1_state.has_lock = will_collide and not psm2_state.has_lock

            if not will_collide or psm1_state.has_lock:
                psm1_state.s = tick(psm1, psm1_state)
            else:
                psm1_state.waiter = FakeWaiter()
        
        if psm2_active:
            will_collide = all(collision_risk(psm1_state, psm2_state)) 
            psm2_state.has_lock = will_collide and not psm1_state.has_lock

            if not will_collide or psm2_state.has_lock:
                psm2_state.s = tick(psm2, psm2_state)
            else:
                psm2_state.waiter = FakeWaiter()
        
        if now // 1 > lastprint:
            print ("PSM1: %s\tPSM2: %s" % (psm1_state.s, psm2_state.s))
            print (collision_risk(psm1_state, psm2_state))
            print (psm1_state.has_lock, psm2_state.has_lock)
            lastprint = now // 1
    return time() - start

def tick(psm, state):
    if state.waiter.is_busy():
        return state.s
    global Z_OFFSET
    #
    # STATE MACHINE
    #
    if state.s == PSMState.s.Standby:
        return PSMState.s.RequestItem

    elif state.s == PSMState.s.RequestItem:
        state.target, fault = dispatch_item(psm)
        if fault:
            # EXIT POINT: No more items to collect.
            state.waiter = FakeWaiter()
            psm.move_jp(np.zeros(6))
            return PSMState.s.Finished
        else:
            print psm.name() + " target: " + str(state.target)
            return PSMState.s.MoveToItem
    
    elif state.s == PSMState.s.MoveToItem:
        print psm.name() + " dist to target:" + str(dist(psm.measured_cp().p, state.target, xy=True))
        if dist(psm.measured_cp().p, state.target, xy=True) < 0.005:
            return PSMState.s.Grab

        state.waiter = traverseToLocation(psm, state.target, Z_OFFSET)
        return state.s #if state.waiter.is_busy() else PSMState.s.Grab

        """
        elif state.s == PSMState.s.Descend:
            state.waiter = PSMSequence([
                psm.jaw.open,
                (move_vertical, (psm, -Z_OFFSET))
            ])
            return PSMState.s.Grab
        """
    elif state.s == PSMState.s.Grab:
        print(psm.name() + " GRABBING!")
        state.waiter = PSMSequence([
            psm.jaw.open,
            (move_vertical, (psm, -Z_OFFSET)),
            psm.jaw.close,
            (move_vertical, (psm, +Z_OFFSET)),
        ])
        return PSMState.s.RequestBowl

        # LEGACY
        #state.waiter = psm.jaw.close()
        #return PSMState.s.Ascend
        """
        elif state.s == PSMState.s.Ascend:
            state.waiter = move_vertical(psm, Z_OFFSET)
            return PSMState.s.RequestBowl
        """
    
    elif state.s == PSMState.s.RequestBowl:
        state.target, fault = dispatch_bowl(psm)
        if fault:
            print("ERROR! No bowl detected!")
            psm.home()
            # EXIT POINT: No bowl detected
            return PSMState.s.Standby
        else:
            return PSMState.s.MoveToBowl

    elif state.s == PSMState.s.MoveToBowl:
        print dist(psm.measured_cp().p, state.target, xy=True)
        if dist(psm.measured_cp().p, state.target, xy=True) <= 0.01:
            return PSMState.s.Release
        state.waiter = traverseToLocation(psm, state.target, Z_OFFSET)
        return state.s

    elif state.s == PSMState.s.Release:
        state.waiter = PSMSequence([
            psm.jaw.open,
            psm.jaw.close
        ])
        return PSMState.s.Home

    elif state.s == PSMState.s.Home:
        # state.waiter = psm.home()  # We don't necessarily want this! Let's just get a new target.
        return PSMState.s.RequestItem


if __name__ == "__main__":
    home_ecm()
    dvrk.ecm
    psm1 = initialize("PSM1")#SafePSM1()
    delay = time()
    while time() - delay < 2.5: # Let the progra settle
        continue
    psm2 = initialize("PSM2")#SafePSM2() 
    delay = time()
    while time() - delay < 2.5: # Let the progra settle
        continue
    
    # Retrieve image of system from camera and extract features 
    # Move this into its own thread later, if desired
    image = capture_system()
    features = extract_system_features(image)
    
    # Sort the items and bowls by their positions such that
    # items closer to psm1 appear on the left side of the list.
    sort_by_y = lambda xyz: xyz[1]
    items = deque(sorted(features["items"], key=sort_by_y))
    bowls = list(sorted(features["bowls"], key=sort_by_y))

    # Start arms. These functions hang the thread.
    #from SafePSM import SafePSM1, SafePSM2
    print "Going home..."
    #print psm1.setpoint_cp()
    # Let the games begin!
    elapsed = mainloop(psm1, psm2)
    print("Task finished. Elpsed time: %ds" % elapsed)

