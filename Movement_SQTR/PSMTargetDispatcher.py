from collections import deque 
class PSMTargetDispatcher:
    
    def __init__(self):
        self._targets = deque()  # Docs specify deque pops and appends 
                                 # are atomic = thread-safe

    def add_targets(self, targets):
        """
        targets: A tuple of x-y-z tuples 
        When targets are added
        """
        # This is a jank solution. It sorts the incoming Deque 
        # by y-value, but doesn't sort any existing targets with it.
        # Instead, it puts negative Y values on the left and positive
        # Y values to the right.
        #
        # Deque structure is [Low Y vals ... High Y vals]
        # Thus, PSM2 pops from right, PSM1 pops from left
        sorter = lambda target: target[1]
        for target in sorted(targets, key=sorter):
            if target[1] > 0: 
                self._targets.append(target)
            else:
                self._targets.appendleft(target)
        
    def target_available(self):
        # Not necessarily thead-safe. Will not crash the system,
        # but the length may be modified during read.
        # Whatever for now :-)
        return len(self._targets)  # Truthy
        
    def request_target_psm1(self):
        return self._targets.popleft()

    def request_target_psm2(self): 
        return self._targets.pop()

