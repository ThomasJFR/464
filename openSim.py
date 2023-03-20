"""
For one-click setting up the CoppeliaSim Simulation.

You need to run 'sudo apt-get install gnome-terminal' one time first.
"""
import os
import time 

# Start roscore in one terminal
os.system("gnome-terminal -e 'bash -c \"roscore; bash\" '")

time.sleep(1)

# Start coppeliasim in another terminal
command = "cd ../../..\n" + "cd /home/fizzer/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04\n" + "./coppeliaSim.sh"
os.system("gnome-terminal -e 'bash -c \"" + command + "; bash\" '")

time.sleep(1)

# Set up json files
command = "cd ../../../../..;" + "cd /home/fizzer/;" + "roscd dvrk_config/console/console_config_patient_side/;" + "rosrun dvrk_robot dvrk_console_json -j console-ECM-PSM1-PSM2_KIN_SIMULATED.json"
os.system("gnome-terminal -e 'bash -c \"" + command + "; bash\" '")


