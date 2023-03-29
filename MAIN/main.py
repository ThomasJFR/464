# MECH 464 Group 1
# This file is the primary file for running the 
# Pick and place operation. It contains the main while loop
# and the system state machine.

# IMPORTS
# Python builtins
from time import time, sleep
from collections import deque
from enum import Enum

# Externals
import dvrk
import PyKDL
import numpy as np

# Created
#import SafePSM
from calibration import calibrate_z
from moves import traverse_to_location, move_vertical, home_ecm
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


def initialize(arm_name):
    """
    This is a function to initialize and home each arm, and to send each arm to the 
    mainloop once initialized.
    """
    # Initialize arm
    p = dvrk.psm(arm_name) #SafePSM(arm_name)
    
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
        "ApproachBowl",
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

def collision_risk(psm1, state1, psm2, state2, as_list=False):
    # Extract positions and targets
    pos1, tgt1 = psm1.measured_cp().p, state1.target
    pos2, tgt2 = psm2.measured_cp().p, state2.target
    #target1 = state1.target
    #target2 = state2.target
    if tgt1 is None or tgt2 is None:
        return [False] if as_list else False

    nearby = lambda p1, p2: dist(p1, p2, xy=True) < 0.03
    conditions = [
        any([
            #target1[1] < target2[1],
            nearby(tgt1, tgt2),
            nearby(pos1, tgt2),
            nearby(pos2, tgt1),
        ]),
        any([
            "Move" in state1.s.name,
            "Move" in state2.s.name
        ])
    ]
    return conditions if as_list else all(conditions)

def mainloop(psm1, psm2):
    # State variables
    psm1_state, psm2_state = PSMState(), PSMState() 
    psm1_active, psm2_active = True,  True

    start = time()
    lastprint = 0
    while psm1_active or psm2_active:
        now = time()
        psm1_active = psm1_state.s is not PSMState.s.Finished
        psm2_active = psm2_state.s is not PSMState.s.Finished

        if psm1_active:
            will_collide = collision_risk(psm1, psm1_state, psm2, psm2_state)
            psm1_state.has_lock = will_collide and not psm2_state.has_lock

            if not will_collide or psm1_state.has_lock:
                psm1_state.s = tick(psm1, psm1_state)
            else:
                psm1_state.waiter = FakeWaiter()
        
        if psm2_active:
            will_collide = collision_risk(psm1, psm1_state, psm2, psm2_state)
            psm2_state.has_lock = will_collide and not psm1_state.has_lock

            if not will_collide or psm2_state.has_lock:
                psm2_state.s = tick(psm2, psm2_state)
            else:
                psm2_state.waiter = FakeWaiter()
        
        if now // 1 > lastprint:
            print ("PSM1: %s\tPSM2: %s" % (psm1_state.s, psm2_state.s))
            #print (collision_risk(psm1_state, psm2_state))
            #print (psm1_state.has_lock, psm2_state.has_lock)
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
        #print psm.name() + " dist to target:" + str(dist(psm.measured_cp().p, state.target, xy=True))
        if dist(psm.measured_cp().p, state.target, xy=True) < 0.005:
            return PSMState.s.Grab

        state.waiter = traverse_to_location(psm, state.target, Z_OFFSET)
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
        #print(psm.name() + " GRABBING!")
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
            return PSMState.s.ApproachBowl

    elif state.s == PSMState.s.ApproachBowl:
        approach_target = state.target
        approach_target[1] += 0.05 *(-1 if psm.name() == "PSM1" else 1)
        
        if dist(psm.measured_cp().p, approach_target, xy=True) <= 0.01:
            return PSMState.s.MoveToBowl

        state.waiter = traverse_to_location(psm, approach_target, Z_OFFSET)
        return state.s

    elif state.s == PSMState.s.MoveToBowl:
        if dist(psm.measured_cp().p, state.target, xy=True) <= 0.01:
            return PSMState.s.Release
        state.waiter = traverse_to_location(psm, state.target, Z_OFFSET)
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
    import argparse
    parser = argparse.ArgumentParser()
    #parser.add_argument("-v", "--verbose", action="store_true", help="Increase verbosity")
    parser.add_argument("-c", "--calibrate", action="store_true", help="Allows user to calibrate the arm before beginning pick-and-place")
    args = parser.parse_args()

    # SR: Initialize the arms
    #home_ecm().wait()
    print("PSM1 homing...")
    psm1 = initialize("PSM1")
    sleep(2.5)
    print("PSM2 homing...")
    psm2 = initialize("PSM2")
    sleep(2.5)

    # SR: Calibrate the z of the table, if requested
    if args.calibrate:
        calibrate_z(psm1)  # Hangs the thread

    # SR: Move the arms out of the way
    #psm1.safe_home().wait()
    #psm2.safe_home().wait()
    # SR: Retrieve image of system from camera and extract features 
    image = capture_system()
    features = extract_system_features(image)

    # SR: Generate features
    # Sort the items and bowls by their positions such that
    # items closer to psm1 appear on the left side of the list.
    sort_by_y = lambda xyz: xyz[1]
    items = deque(sorted(features["items"], key=sort_by_y))
    bowls = list(sorted(features["bowls"], key=sort_by_y))

    #print psm1.setpoint_cp()
    # Let the games begin!

    elapsed = mainloop(psm1, psm2)
    print("Task finished. Elpsed time: %ds" % elapsed)

