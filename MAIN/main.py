# MECH 464 Group 1
# This file is the primary file for running the 
# Pick and place operation. It contains the main while loop
# and the system state machine.

# IMPORTS
# Python builtins
from time import time, sleep
from collections import deque

# Externals
import dvrk
import PyKDL
import numpy as np

# Created
#import SafePSM
from calibration import calibrate_z
from moves import traverse_to_location, move_vertical, home_ecm
from utils import FakeWaiter, PSMSequence, dist
from vision import capture_system, extract_system_features
from collision import collision_risk
from psm_state_machine import PSMState

items = deque()
bowls = list()
Z_OFFSET = 0.05

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


def initializePSM(arm_name):
    """
    This is a function to initialize and home each arm, and to send each arm to the 
    mainloop once initialized.
    """
    # Initialize arm
    p = dvrk.psm(arm_name) #SafePSM(arm_name)
    
    # Home the arm
    p.enable()
    p.home()
    sleep(2)  # Necessary for simulator
    return p

def initializeECM(arm_name):
    """
    This is a function to initialize and home each arm, and to send each arm to the 
    mainloop once initialized.
    """
    # Initialize arm
    e = dvrk.ecm(arm_name)

    # Home the arm
    e.enable()
    e.home()
    sleep(2)
    return e 

def mainloop(args):
    # State variables
    psm1, psm2 = args["PSM1"], args["PSM2"]
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
        
        if now - lastprint > 0.5:
            print ("PSM1: %s\tPSM2: %s" % (psm1_state.s, psm2_state.s))
            #print (collision_risk(psm1_state, psm2_state))
            #print (psm1_state.has_lock, psm2_state.has_lock)
            lastprint = now
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

    elif state.s == PSMState.s.Grab:
        #print(psm.name() + " GRABBING!")
        state.waiter = PSMSequence([
            psm.jaw.open,
            (move_vertical, (psm, -Z_OFFSET)),
            psm.jaw.close,
            (move_vertical, (psm, +Z_OFFSET)),
        ])
        return PSMState.s.RequestBowl

    elif state.s == PSMState.s.RequestBowl:
        state.target, fault = dispatch_bowl(psm)
        if fault:
            print("ERROR! No bowl detected!")
            psm.home()
            # EXIT POINT: No bowl detected
            return PSMState.s.Standby
        
        if psm.name() == "PSM1":
            return PSMState.s.MoveToBowl
        else:
            return PSMState.s.ApproachBowl

    elif state.s == PSMState.s.ApproachBowl:
        approach_target = np.array([x for x in state.target])
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
        # In coppeliasim, PSM1 crashes if we go to the blue bowl.
        # TODO Fix this later
        if psm.name() == "PSM1":
            return PSMState.s.Home
        else:
            return PSMState.s.BackOffBowl
 
    elif state.s == PSMState.s.BackOffBowl:
        approach_target = np.array([x for x in state.target])
        approach_target[1] += 0.05 *(-1 if psm.name() == "PSM1" else 1)
        
        if dist(psm.measured_cp().p, approach_target, xy=True) <= 0.01:
            return PSMState.s.Home

        state.waiter = traverse_to_location(psm, approach_target, Z_OFFSET)
        return state.s

    elif state.s == PSMState.s.Home:
        # state.waiter = psm.home()  # We don't necessarily want this! Let's just get a new target.
        return PSMState.s.RequestItem

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    #parser.add_argument("-v", "--verbose", action="store_true", help="Increase verbosity")
    parser.add_argument("-c", "--calibrate", action="store_true", help="Allows user to calibrate the arm before beginning pick-and-place")
    args = parser.parse_args()

    e = initializeECM("ECM")
    sleep(2)
    # SR: Initialize the arms
    #home_ecm().wait()
    print("PSM1 homing...")
    psm1 = initializePSM("PSM1")
    print("PSM2 homing...")
    psm2 = initializePSM("PSM2")

    print("Homing ECM")
    home_ecm(e).wait()
    
    # SR: Calibrate the z of the table, if requested
    if args.calibrate:
        print("------------------------------------------")
        print("-  TOUCH PSM1 TIP TO TABLE TO CALIBRATE  -")
        print("------------------------------------------")
        z_calibrated = calibrate_z(psm1)  # Hangs the thread

    # SR: Move the arms out of the way
    #psm1.safe_home().wait()
    #psm2.safe_home().wait()
    # SR: Retrieve image of system from camera and extract features 
    image = capture_system()
    features = extract_system_features(image)

    # SR: Generate features
    # Sort the items and bowls by their positions such that
    # items closer to psm1 appear on the left side of the list.
    # TODO Fix the bad global pattern
    sort_by_y = lambda xyz: xyz[1]
    items = deque(sorted(features["items"], key=sort_by_y))
    bowls = list(sorted(features["bowls"], key=sort_by_y))
    if args.calibrate:
        for i in range(len(items)):
            items[i][2] = z_calibrated 
        for i in range(len(bowls)):
            bowls[i][2] = z_calibrated 
    # Let the games begin!
    elapsed = mainloop({
        "PSM1": psm1,
        "PSM2": psm2,
    })
    print("Task finished. Elpsed time: %ds" % elapsed)

