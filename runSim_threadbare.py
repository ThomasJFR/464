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
    features["items"] = [
        np.array([-1.527791262, 0.05018193647, 0.674774766]),
        np.array([-1.516814113, -0.04662011936, 0.6747748256]),
        np.array([-1.547450662, -0.05359531567, 0.674774766]),
        np.array([-1.536615372, 0.003250310896, 0.674774766]),
        np.array([-1.51279664, 0.02789815143, 0.674774766]),
        np.array([-1.550005555, -0.02500112727, 0.6747747064]),
    ]
    features["bowls"] = [
        np.array([-0.4560598135, 0.6324006319, 0.8911616802]),
    ]
    return features

def dispatch(p):
    """
    This function  interfaces with the computer vision part of the system and decides
    which object to try to pick up next based on which arm is asking, and whether it has
    the mutex.
    """
    global items, bowls

    if len(items) == 0:
        return None, None, True

    # Get the closest target and bowl
    is_psm1 = (p.name == "PSM1")  # For readability
    item = tems.popleft() if is_psm1 else items.pop()
    bowl = bowls[0] if is_psm1 else bowls[-1]  # bowls[0] is closest to psm1; bowls[-1] to psm2

    return item, bowl, False

def traverseToLocation(p, location, z_offset):
    """
    Function that moves the gripper from one position to another based on 
    cartesian commands.
    """
    goal = p.setpoint_cp()
    goal.p[0] = location[0] # Set x position
    goal.p[1] = location[1] # Set y position
    goal.p[2] = location[2] + zoffset # Set z position
    return p.move_cp(goal)

def move_vertical(p, z):
    goal = p.setpoint_cp()
    goal.p[2] += z
    return p.move_cp(goal)


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
    return 

fake_waiter = {
    "wait": (lambda: return), 
    "is_busy": (lambda: return False)
}


class PSMState:
    s = Enum("PSM States", [
        "Standby",
        "Request",
        "MoveToItem", 
        "Descending",
        "Grabbing",
        "Ascending",
        "MoveToBowl",
        "Releasing",
        "Home"
    ])
    
    def __init__(self):
        self.s = PSMState.s.Standby 
        self.item = None
        self.bowl = None
        self.waiter = fake_waiter

class PSMPipeline:
    def __init__(self, actions):
        self.__actions = deque(actions)
        self.__waiter = fake_waiter

    def tick(self):
        if not self.__waiter.is_busy()
            self.__waiter = self.__actions.popleft()

    def is_busy(self):
        return (len(self.__actions) == 0) and (not self.__waiter.is_busy())
    
def mainloop(psm1, psm2):
    # State variables
    psm1_state, psm2_state = PSMState(), PSMState() 
    psm1_active, psm2_active = True, True

    start = time()
    while psm1_active or psm2_active:
        psm1_active = tick(psm1, psm1_state)
        psm2_active = tick(psm2, psm2_state)
    return time() - start

def tick(psm, state):
    if state.waiter.is_busy():
        return True
    
    global Z_OFFSET
    # State switch
    if state.s == PSMState.s.Standby:
        state.target, state.bowl, fault = dispatch(psm)
        if fault:
            # EXIT POINT: No more items to collect.
            return False
        else:
            state.s = PSMState.s.MoveToItem
    
    elif state.s == PSMState.s.MoveToItem:
        state.waiter = traverseToLocation(psm, state.item, Z_OFFSET)
        state.s = PSMState.s.Descending
    
    elif state.s == PSMState.s.Descending:
        psm.jaw.open().wait()  # Absorb this time
        state.waiter = move_vertical(p, -Z_OFFSET)
        state.s = PSMState.Grabbing
    
    elif state.s == PSMState.s.Grabbing:
        state.waiter = psm.jaw.close()
        state.s = PSMState.s.Ascending

    elif state.s == PSMState.s.Ascending:
        state.waiter = move_vertical(p, Z_OFFSET)
        state.s = PSMState.s.MoveToBowl

    elif state.s == PSMState.s.MoveToBowl:
        state.waiter = traverseToLocation(psm, state.bowl, Z_OFFSET)
        state.s = PSMState.s.Releasing

    elif state.s == PSMState.s.Releasing:
        psm.jaw.open().wait()  # Absorb this time
        state.waiter = psm.jaw.close()
        state.s = PSMState.s.Home

    elif state.s == PSMState.s.Home:
        # state.waiter = psm.home()  # We don't necessarily want this! Let's just get a new target.
        state.s = PSMState.s.Standby
    
    return True

if __name__ == "__main__":
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
    psm1 = initialize("PSM1")
    psm2 = initialize("PSM2")

    # Let the games begin!
    elapsed = mainloop(psm1, psm2)
    print("Task finished. Elpsed time: %ds" % elapsed)

