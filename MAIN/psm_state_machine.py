from enum import Enum
from utils import FakeWaiter
class PSMState:
    s = Enum("PSM States", [
        "Standby",
        "RequestItem",
        "MoveToItem", 
        "Descend",
        "Grab",
        "Ascend",
        "RequestBowl",
        "ApproachBowl",
        "MoveToBowl",
        "Release",
        "BackOffBowl",
        "Home",
        "Finished"
    ])
    
    def __init__(self):
        self.s = PSMState.s.Standby 
        self.s_next = PSMState.s.Standby
        self.target = None
        self.waiter = FakeWaiter()
        self.has_lock = False
        self.zcal = None
