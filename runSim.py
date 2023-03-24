#%%
# MECH 464 Group 1
# This file is the primary file for running the 
# Pick and place operation. It contains the main while loop
# and the system state machine.

# IMPORTS
import numpy as np
import threading
import dvrk
import PyKDL


def traverseToLocation(p, delta) -> None:
    """
    Function that moves the gripper from one position to another based on 
    cartesian commands.
    """
    # Go to water
    goal.p[0] = x0 + delta[0] # Set x position
    goal.p[1] = y0 + delta[1] # Set y position
    goal.p[2] = z0 + delta[2] + z_offset # Set z position
    p.move_cp(goal).wait()

    return

def grabObject(p, zoffset: float) -> None:
    """
    Function that opens the gripper, descends to the object, closes the gripper, and 
    ascends back to a traverse height to attempt to grab and pick up an object.
    """
    # 1. Open gripper
    p.jaw.open().wait() 

    # 2. Descend
    goal.p[2] = z0 - z_offset # Set z position
    p.move_cp(goal).wait()

    # 3. Close gripper
    p.jaw.close().wait()
    
    # 4. Ascend
    goal.p[2] = z0 + z_offset # Set z position
    p.move_cp(goal).wait()

    return



def releaseObject(p) -> None:
    """
    Function that opens and closes the gripper to release an object it is holding.
    """
    p.jaw.open().wait() # Open the jaw and wait
    p.jaw.close().wait() # Close the jaw and wait

    return

def goHome(p) -> None:
    """
    Function to send the arm home.
    """
    
    return


def dispatch(p, hasMutex: bool) -> None:
    """
    This function  interfaces with the computer vision part of the system and decides
    which object to try to pick up next based on which arm is asking, and whether it has
    the mutex.
    """
    fault = False

    # Define any bounding planes

    # Retrieve image from the camera

    # Identify objects in the image

    # If there are no objects remaining
    #   fault = True
    #   item = None
    #   bowl = None

    # else:
    #   Get a list of the x, y, z coordinates for the objects and bowls in the image
    #   Determine which subset of the workspace this arm has access to (and therefore which objects it is allowed to pick up)
    #   If there are no objects in the restricted workspace:
    #      fault = True
    #      item = None
    #      bowl = None
    
    #   else:
    #      Decide which object and bowl this arm should to go
    #      item = [x, y, z] coordinates of the object
    #      bowl = [x, y, z] coordinates of the bowl   

    return item, bowl, fault


def mainloop(p) -> None: # p is the arm object
    """
    Fruit loops :) 
    """
    # The following is PSUEDOCODE until we figure 
    # out a better implementation
    faultCount = 0
    while(faultCount < 5): # Continue to loop until the dispatcher reports some number of faults
        # Try to acquire the mutex
        with mutex:
            # Call te dispatcher with the mutex
            item, bowl, fault = dispatch(p, mutex)

            if not fault: # If there are objects available within reach
                traverseToLocation(p, item)
                grabObject(p)
                traverseToLocation(p, bowl)
                releaseObject(p)
        
        else:
            # Call the dispatcher without the mutex
            item, bowl, fault = dispatch(p, mutex)

            if not fault: # If there are objects available within reach
                traverseToLocation(p, item)
                grabObject(p)
                traverseToLocation(p, bowl)
                releaseObject(p)

    # After there are no more objects for the arm to pick up:
    p.home()

    return 


def initialize(armName: str) -> None:
    """
    This is a function to initialize and home each arm, and to send each arm to the 
    mainloop once initialized.
    """
    # Initialize arm
    p = dvrk.psm(armName)

    # Home the arm
    p.enable()
    p.home()

    # Send the arm to the mainloop
    mainloop(p)

    return 


if __name__ == "__main__":
    # Create 2 threads, one for each arm
    # PSM1 thread
    PSM1_Thread = threading.Thread(target = initialize, args = ('PSM1',))
    
    # PSM1 thread
    PSM2_Thread = threading.Thread(target = initialize, args = ('PSM2',))
    
    # Creating a mutex lock for protecting the shared area of the workspace
    mutex = threading.Lock()
    mutex2 = threading.Semaphore(1)
    
    # Start both threads
    PSM1_Thread.start()
    PSM2_Thread.start()

    # Once the threads return, join
    PSM1_Thread.join()
    PSM2_Thread.join()    
