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

        self.__waiter = None 
        for action, arg in zip(actions, args):
            self.__actions.append(action)
            self.__args.append(arg)

    def tick(self):
        """
        Returns whether the ticker has run down
        """
        if self.__waiter and self.__waiter.is_busy():
            return False
        elif len(self.__actions) > 0:
            action_fun = actions.pop()
            args = self.__args.pop()
            self.__waiter = action_fun(args) 
        else:
            return True

    def is_busy():
        return not tick()

    def wait(self):
        while self.is_busy()
            continue

class FakeWaiter:
    def wait(self):
        return 
    
    def is_busy(self): 
        return False

