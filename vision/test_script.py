import dvrk
import numpy as np
#%%
# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')
e = dvrk.ecm('ECM')
#%%
# p.move_jp(np.array([0.5,0.5,1,0.5,0.5,0.5])).wait()
e.move_jp(np.array([0.5,0.5,0.5,0.5]))
# %%
