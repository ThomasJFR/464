import dvrk
import numpy as np
from VRep import sim as vrep
import cv2


#Ensure there are no outstanding simulations running
vrep.simxFinish(-1)

#Open  connection to the simulator
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

visionSensor = 'Vision_sensor_left'

[returnCode,Vision_sensor]=vrep.simxGetObjectHandle(clientID,visionSensor,vrep.simx_opmode_blocking);
vrep.simxGetVisionSensorImage2(clientID,Vision_sensor,0,vrep.simx_opmode_streaming);
while (clientID!=-1):
    [returnCode,resolution,Image]=vrep.simxGetVisionSensorImage2(clientID,Vision_sensor,1,vrep.simx_opmode_buffer);
    if (returnCode==vrep.simx_return_ok):
        print("There's an image")
        imshow(Image)
    end
end

"""
if clientID!=-1:
    print 'Connected to remote API server'
    
    # get vision sensor objects
    res, v1 = vrep.simxGetObjectHandle(clientID, visionSensor, vrep.simx_opmode_oneshot_wait)
    
    # get first image
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    
    while (vrep.simxGetConnectionId(clientID) != -1):
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok:
            print "image OK"
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
            cv2.imshow('image',img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == vrep.simx_return_novalue_flag:
            print "no image yet"
            pass
        else:
          print err
else:
  print "Failed to connect to remote API Server"
  vrep.simxFinish(clientID)

cv2.destroyAllWindows()

class Vision(dvrk.psm):
    def __init__(self, arm_name="PSM1"):
        super(dvrk.psm, self).__init__(arm_name) 
        #self.xbound = [0, 0]
        self.ybound = [0, 0]
        #self.zbound = [0, 0]

    def safe_home(self):
        
        #Retracts the extender, then moves to the vertical position.
        
        # Retract
        goal = self.setpoint_jp()
        goal[2] = 0
        self.move_jp(goal).wait()
         
        # Move home
        self.move_jp(np.zeros(6)).wait()
"""
