import dvrk
import numpy as np

def move_vertical(manipulator):
    return manipulator.move_jp(np.zeros(6))

def move_vertical_safely(manipulator):
    goal = manipulator.setpoint_jp()
    goal[2] = 0
    manipulator.move_jp(goal).wait()
    return move_vertical(manipulator)

def move_cp(manipulator, dx, dy, dz):
    goal = manipulator.setpoint_cp()
    goal.p[0] += dx
    goal.p[1] += dy
    goal.p[2] += dz
    return manipulator.move_cp(goal)
    
def knock_over_blue_bowl(manipulator):
    move_vertical_safely(manipulator).wait()
    move_cp(manipulator, 0, 0.02, -0.12).wait()
    move_cp(manipulator, 0.20, 0, 0).wait()

def crush_blue_bowl(m1, m2):
    ## UNFINISHED 
    # Move to base
    m1wait = move_vertical_safely(m1)
    move_vertical_safely(m2).wait()
    m1wait.wait()

    move_cp(m1, 0, 0.02, -0.12).wait()
    move_cp(m2, 0, -0.05, -0.12).wait()
    

