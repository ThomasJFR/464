#%%
"""
This is a function that generates a trajectory for the DVRK robot.

INPUTS:


OUTPUTS:

"""

# Useful Coppeliasim Lua Commands:
# simGetObjectHandle("<NAME>")
# simGetObjectPosition(of_handle1, wrt_handle2)
#
# World handle: 8
# Blue bowl handle: 255
# PSM1 end handle: 224 
# Water1 handle: 253
# Water2 handle: 229
# Water3 handle: 231
# Water4 handle: 233
# Water5 handle: 235
# Water6 handle: 237

import numpy as np
import matplotlib.pyplot as plt
import dvrk
import PyKDL

# Initialize arm
p = dvrk.psm('PSM1')

# Home
p.enable()
p.home()

#%% Generate trajectory points
N = 20
scale = 0.01
t = np.linspace(0, scale, N)
xvals = np.cos(t / scale**2) * scale
yvals = np.sin(t / scale**2) * scale
zvals = np.copy(t)

# Move
# start position
home = p.setpoint_cp() # saving home position
goal = p.setpoint_cp()
x0 = goal.p[0]
y0 = goal.p[1]
z0 = goal.p[2]
z_offset = 0.05

# Positions below are measured from world coord frame in HOME position
psm1_pos = np.array([-1.512597919, -0.08245876431, 0.6853816509])
bluebowl_pos = np.array([-0.4560598135, 0.6324006319, 0.8911616802])
water1_pos = np.array([-1.527791262, 0.05018193647, 0.674774766])
water2_pos = np.array([-1.516814113, -0.04662011936, 0.6747748256])
water3_pos = np.array([-1.547450662, -0.05359531567, 0.674774766])
water4_pos = np.array([-1.536615372, 0.003250310896, 0.674774766])
water5_pos = np.array([-1.51279664, 0.02789815143, 0.674774766])
water6_pos = np.array([-1.550005555, -0.02500112727, 0.6747747064])

# Put objects in a list for iterating
objects = [water1_pos, water2_pos, water3_pos, water4_pos, water5_pos, water6_pos]

# Get the delta to the target
bowlDelta = bluebowl_pos - psm1_pos

# First move up to avoid later collisions
goal.p[2] = z0 + z_offset # Set z position
p.move_cp(goal).wait()

# Now grab each water
for idx, obj in enumerate(objects):
    # Get delta to object
    delta = obj - psm1_pos

    # Go to water
    goal.p[0] = x0 + delta[0] # Set x position
    goal.p[1] = y0 + delta[1] # Set y position
    goal.p[2] = z0 + delta[2] + z_offset # Set z position
    p.move_cp(goal).wait()

    # Go down and grab
    goal.p[2] -= z_offset # Set z position
    p.move_cp(goal).wait()  

    # Go up
    goal.p[2] += z_offset # Set z position
    p.move_cp(goal).wait()

    # Go back home
    goal.p[0] = x0 # Set x position
    goal.p[1] = y0 # Set y position
    goal.p[2] = z0 + z_offset # Set z position
    p.move_cp(goal).wait()

    # # Go to target
    # goal.p[0] = x0 + bowlDelta[0] # Set x position
    # goal.p[1] = y0 + bowlDelta[1] # Set y position
    # goal.p[2] = z0 + bowlDelta[2] + z_offset # Set z position
    # p.move_cp(goal).wait()

    # Drop water
    p.jaw.open().wait()
    p.jaw.close().wait()


# goal.p[0] = x0 + delta[0] # Set x position
# goal.p[1] = y0 + delta[1] # Set y position
# goal.p[2] = z0 + delta[2] # Set z position

# p.move_cp(goal).wait()


