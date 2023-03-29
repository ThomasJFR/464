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
    goal.p[2] = location[2] + z_offset # Set z position
    return p.move_cp(goal)

def move_vertical(p, z):
    goal = p.setpoint_cp()
    goal.p[2] += z
    return p.move_cp(goal)


