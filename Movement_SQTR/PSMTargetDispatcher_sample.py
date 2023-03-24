import numpy as np
import PSMTargetDispatcher

dispatcher = PSMTargetDispatcher()
targets = [
    np.array([-1.527791262, 0.05018193647, 0.674774766]),
    np.array([-1.516814113, -0.04662011936, 0.6747748256]),
    np.array([-1.547450662, -0.05359531567, 0.674774766]),
    np.array([-1.536615372, 0.003250310896, 0.674774766]),
    np.array([-1.51279664, 0.02789815143, 0.674774766]),
    np.array([-1.550005555, -0.02500112727, 0.6747747064]),
]
dispatcher.add_targets(targets)

# Within some PSM1 thread,
# where the PSM1 wants to get a new target:
if dispatcher.targets_available():
    x, y, z = dispatcher.request_target_psm1()

# And for PSM2
if dispatcher.targets_available():
    x, y, z = dispatcher.request_target_psm2()

