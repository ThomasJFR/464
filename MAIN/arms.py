import dvrk
from time import sleep
import numpy as np
from moves import safe_retract
from utils import PSMSequence

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

    p.trajectory_j_set_ratio(0.65)
    safe_retract(p).wait()
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

    e.trajectory_j_set_ratio(0.05)

    return e 
