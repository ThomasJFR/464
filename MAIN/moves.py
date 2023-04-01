import numpy as np
from utils import PSMSequence
def home_ecm(e):
    # Test file for ECM home script	
	goal = e.measured_jp()
	goal[1] = -0.54
	goal[2] = 0.0
	
	return e.move_jp(goal)


print("ECM is homed.")

def traverse_to_location(p, location, z_offset):
    """
    Function that moves the gripper from one position to another based on 
    cartesian commands.
    """
    goal = p.setpoint_cp()
    goal.p[0] = location[0] # Set x position
    goal.p[1] = location[1] # Set y position
    #goal.p[2] = location[2] + z_offset # Set z position
    goal.p[2] = location[2] - z_offset # Set z position
    return p.move_cp(goal)

def move_vertical(p, z):
    goal = p.setpoint_cp()
    #goal.p[2] += z # REMOVED FOR DVRK
    goal.p[2] -= z
    return p.move_cp(goal)

def safe_retract(p):
    gripper_aligned = p.setpoint_jp()
    gripper_aligned[4] = 0
    gripper_aligned[5] = 0
    retract_arm = p.setpoint_jp()
    retract_arm[2] = 53.5 / 1000
    retract_arm[4] = 0
    retract_arm[5] = 0
    arm_to_vert = p.setpoint_jp()
    arm_to_vert[0] = 0
    arm_to_vert[1] = 0
    arm_to_vert[3] = 0
    arm_to_vert[4] = 0
    arm_to_vert[5] = 0
    arm_to_vert[2] = 53.5 / 1000
    return PSMSequence([
        (p.move_jp, [gripper_aligned]),
        (p.move_jp, [retract_arm]),
        (p.move_jp, [arm_to_vert])
    ])

