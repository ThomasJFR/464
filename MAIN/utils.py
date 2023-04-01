from collections import deque
from collections import Sequence

class PSMSequence:
    """
    Protothreadable waiter that executes a sequence of actions.
    """
    def __init__(self, sequence, args=None):
        actions = list()
        if args is None:
            # Either no arguments are provided, or
            # sequence is a 2D action-args list.
            args = list()
            for x in sequence:
                list_like = isinstance(x, Sequence)
                actions.append(x[0] if list_like else x)
                args.append(x[1] if list_like else [])
        else:
            actions = sequence
            args = [arg if isinstance(arg, Sequence) else (arg,) for arg in args]
        self.__actions = deque()
        self.__args = deque()
        self.__waiter = None 
        for action, arg in zip(actions, args):
            self.__actions.appendleft(action)
            self.__args.appendleft(arg)

    def tick(self):
        """
        Returns whether the ticker has run down
        """
        if self.__waiter is None:
            self.next()
            return False

        busy = self.__waiter.is_busy()
        if busy: 
            return False
        elif len(self.__actions) > 0:
            self.next()
            return False
        else:
            return True

    def next(self):
        action_fun = self.__actions.pop()
        args = self.__args.pop()
        self.__waiter = action_fun(*args) 

    def is_busy(self):
        #self.tick()
        return not self.tick()#(self.__waiter.is_busy() or len(self.__actions) > 0)

    def wait(self):
        while self.is_busy():
            continue

class FakeWaiter:
    def wait(self):
        return 
    
    def is_busy(self): 
        return False

from PyKDL import Vector
import numpy as np
def dist(xyz1, xyz2, xy=False):
    if isinstance(xyz1, np.ndarray):
        xyz1 = Vector(*xyz1)
    if isinstance(xyz2, np.ndarray):
        xyz2 = Vector(*xyz2)
    if xy:
        xyz1.z(0)
        xyz2.z(0)
    return (xyz1 - xyz2).Norm()

def dist_from_line(P, lineP1, lineP2):
    # See: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
    x0, x1, x2 = P.x(), lineP1.x(), lineP2.x() 
    y0, y1, y2 = P.x(), lineP1.y(), lineP2.y()
    numerator = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
    denominator = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    return numerator/denominator
