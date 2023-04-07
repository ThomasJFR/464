# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# import object detector script
import object_detector as detector
# import camera calibrator script
import camera_calibrator as calibrator
# import other useful packages
import numpy as np
import matplotlib.pyplot as plt

# Instantiate CvBridge
bridge = CvBridge()

def main():
    rospy.init_node('image_listener')

    platform = raw_input("coppelia OR dvrk: ")
    cameraChoice = raw_input("left OR right: ")

    # define image topic
    image_topic = imageTopic(platform,cameraChoice)
    
    # setup subscriber, define callback to save image
    sub = rospy.Subscriber(image_topic, Image, saveImage)
    rospy.sleep(0.1) # pause script to save image
    sub.unregister() # unsubscribe

    # extract location of red objects
    img = cv2.imread('visionDebuggerImages/cameraImageDebugger.png',0) # load debugger image
    # img = cv2.imread('visionDebuggerImages/sampleImage4.png',0) # load debugger image
    # img = cv2.imread('cameraImage.png',0) # load actual image
    x,y = objectLocator(img)

    return x,y

def imageTopic(platform,cameraChoice):
    image_topic = "N/A"

    # define image topic
    if platform == "coppelia":
        if cameraChoice == "left":
            image_topic = "/stereo/left/image_raw"
        elif cameraChoice == "right":
            image_topic = "/stereo/right/image_raw"
        else:
            print("Invalid camera option.")
    elif platform == "dvrk":
        if cameraChoice == "left":
            image_topic = "/dVRK/left/decklink/camera/image_raw"
        elif cameraChoice == "right":
            image_topic = "/dVRK/right/decklink/camera/image_raw"
        else:
            print("Invalid camera option.")        
    else:
        print("Invalid platform option.")
        
    print("rostopic: " + image_topic)
    return image_topic

def saveImage(msg):
    print("Received an image")
    try:        
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8") # Convert ROS Image message to OpenCV2
    except CvBridgeError, e:
        print(e)
    else:
        cv2.imwrite('cameraImage.png', cv2_img) # save image
        print("Image saved!")


def objectLocator(img):
    # Undistort image from calibration
    img_undistorted = calibrator.main(img)
    
    # Give image to detector to locate objects in image frame
    cx,cy = detector.main(img_undistorted)

    # Transform coordinates such that origin is at centre and is in middle of the image
    # TO DO
    # convert cx and cy to x and y

    x = 0
    y = 0
    return x,y

if __name__ == '__main__':
    main()