import dvrk
import numpy as np

class SafePSM1(dvrk.psm):
    def __init__(self, autostart=True, arm_name="PSM1"):
        super(dvrk.psm, self).__init__(arm_name) 
        #self.xbound = [0, 0]
        self.ybound = [0, 0]
        #self.zbound = [0, 0]

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
        goal = self.setpoint_jp()
        goal[2] = 0
        self.move_jp(goal).wait()
         
        # Move home
        self.move_jp(np.zeros(6)).wait()

    def set_safezone(self):
        """
        Creates a safezone based on the current PSM position.
        The safezone is meant to keep the arm angle from "crossing" the
        middle dividing plane.
        """
        pos = self.setpoint_cp().p
        self.ybound[0] = pos[1]
        #self.ybound[1] = 5  # TODO bounding function 
        
    def safe_move_dcp(self, dx=0, dy=0, dz=0):
        """
        Move in cartesian space by the specified amounts.
        Moves if the movement is permitted within the safezone.
        """
        target = self.setpoint_cp()
        x,y,z = target.p
        if not (self.ybound[0] <= (y + dy) <= self.ybound[1]):
            print("Requested move is beyond the safezone!")
            return FailWaiter()

        target.p[0] += dx
        target.p[1] += dy
        target.p[2] += dz
        return self.move_cp(target)
    
class PSMBoundary:
    def __init__(self, PSM1, PSM2):
        self._PSM1 = PSM1
        self._PSM2 = PSM2
        self._bound_plane = 0  # y coordinate

    def set_bounding_plane(self, y):
        if y > 0.08:
            print("Outside of dextrous workspace!")

        self._bound_plane = y
        self._PSM1.ybound[1] = y
        self._PSM2.ybound[1] = y

    def get_bound(self, dim=None):
        return bound_plane

class FailWaiter:
    def wait(self):
        return False

