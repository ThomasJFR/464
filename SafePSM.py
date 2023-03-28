from dvrk import psm
import numpy as np
from math import radians

class SafePSM(psm):
    def __init__(self, arm_name, autostart=True):
        #super(dvrk.psm, self).__init__(arm_name)
        self.xbounds, self.ybounds, self.zbounds = [], [], []
        psm.__init__(self, arm_name)

        if autostart:
            self.start()

    def start(self):
        self.enable()
        self.home()

    def safe_home(self):
        """
        Retracts the extender, then moves to the vertical position.
        """
        # Retract
        retract = self.setpoint_jp()
        retract[2] = 0
        # TODO Add an angle
        self.move_jp(retract).wait()
         
        # Move home
        home = np.zeros(6) 
        home[2] = -0.045  # Moves the retractor outside the sheath
        self.move_jp(np.zeros(6)).wait()
    
    def set_safezone(self, xbounds=[], ybounds=[], zbounds=[]):
        # The legacy implementation
        if not any([xbounds, ybounds, zbounds]):
            return self.OLD_set_safezone()
        
        # The actual implementation
        if xbounds:
            self.xbounds = xbounds
        if ybounds:
            self.ybounds = ybounds
        if zbounds:
            self.zbounds = zbounds

    def in_safezone(self, xyz):
        x, y, z = xyz
        within = lambda a, abounds: (abounds[0] <= a <= abounds[1])

        return all([
            within(x, self.xbounds) if self.xbounds else True,
            within(y, self.ybounds) if self.ybounds else True,
            within(z, self.zbounds) if self.zbounds else True,
        ])

    def constrain_to_safezone(self, xyz):
        x, y, z = xyz
        within = lambda a, abounds: (abounds[0] <= a <= abounds[1])
        if self.xbounds and not within(x, self.xbounds):
            x = self.xbounds[0] if x < self.xbounds[0] else self.xbounds[1]
        if self.ybounds and not within(y, self.ybounds):
            y = self.ybounds[0] if y < self.ybounds[0] else self.ybounds[1]
        if self.zbounds and not within(z, self.zbounds):
            z = self.zbounds[0] if z < self.zbounds[0] else self.zbounds[1]
        return [x, y, z]

    def get_safezone(self):
        return self.xbounds, self.ybounds, self.zbounds
    
    def OLD_set_safezone(self):
        """
        Creates a safezone based on the current PSM position.
        The safezone is meant to keep the arm angle from "crossing" the
        middle dividing plane.
        """
        print("Warning: USING LEGACY IMPLEMENTATION of set_safezone!\n"
              "\tThe legacy implementation uses the current position of the arm.\n"
              "\tThe latest implementation takes parameters set_safezone(xbounds, ybounds,zbounds)")
        pos = self.setpoint_cp().p
        self.ybounds[0] = pos[1]
        
    def open(self, angle=radians(60)):
        """
        Opens the jaws fully or by the specified amount in radians
        Unstickies the jaws too
        """
        return self.jaw.open(angle)

    def close(self):
        """
        Closes the jaws and makes them sticky
        """
        return self.jaw.close()

    def move_dcp(self, dx=0, dy=0, dz=0):
        """
        Move the end-effector in cartesian space by a specified amount 
        relative to the current cartesian position. 
        """
        target = self.setpoint_cp()
        #target.p[0] += dx
        #target.p[1] += dy
        #target.p[2] += dz
        target.p += np.array([dx, dy, dz])
        return self.move_cp(target)
   
    def safe_move_pos(self, x=None, y=None, z=None, aggressive=True):        
        target = self.setpoint_cp()
        target.p[0] = target.p[0] if x is None else x
        target.p[1] = target.p[1] if y is None else y
        target.p[2] = target.p[2] if z is None else z
        if self.in_safezone(target.p):  # Move is safe! 
            return self.move_cp(target)
        
        # If the move was not safe, determine what to do
        print("Requested move is beyond the safezone!")
        if aggressive:
            new_x, new_y, new_z = self.constrain_to_safezone(target.p)
            target.p[0] = new_x
            target.p[1] = new_y
            target.p[2] = new_z 
            return self.move_cp(target)
        else:
            return FailWaiter()

    def safe_move_dpos(self, dx=0, dy=0, dz=0, aggressive=True):
        """
        As with move_dcp, but only works if the movement is within the safezone.
        If aggressive is True, the arm will go as close to its final position as it can.
        """
        x, y, z = self.setpoint_cp().p
        return self.safe_move_pos(x + dx, y + dy, z + dz)

# Helpers to make the acquisiton process quicker
def SafePSM1(): return SafePSM("PSM1")
def SafePSM2(): return SafePSM("PSM2")
def SafePSM3(): return SafePSM("PSM3")       

class FailWaiter:
    def wait(self):
        return False

