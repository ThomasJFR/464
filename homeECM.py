# Test file for ECM home script
from dvrk import ecm
from time import sleep

e = ecm("ECM")
e.enable()
e.home()

sleep(2.5)

goal = e.measured_jp()
goal[1] = -0.54
goal[2] = 0.0

print("Homing the ECM.")
e.move_jp(goal).wait()
print("ECM is homed.")


