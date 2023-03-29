# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()
imageCounter = 0

def main():
    global imageCounter
    rospy.init_node('image_listener')
    platform = raw_input("coppelia OR dvrk: ")
    cameraChoice = raw_input("left OR right: ")
    
    # define image topic
    image_topic = imageTopic(platform,cameraChoice)

    print("Total images captured: "+ str(imageCounter))    
    continueCapture = raw_input("Continue taking images? y/n: ")

    # loop until user completes calibration
    while continueCapture == "Y" or continueCapture == "y":
        imageCounter+=1

        # setup subscriber, define callback to save image
        sub = rospy.Subscriber(image_topic, Image, saveImage)
        rospy.sleep(0.1) # pause script to save image
        sub.unregister() # unsubscribe
        print("Total images captured: "+ str(imageCounter))

        continueCapture = raw_input("Continue taking images? y/n: ")

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
    global imageCounter
    print("Received an image")
    try:        
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8") # Convert ROS Image message to OpenCV2
        cv2_img = cv2.rotate(cv2_img,cv2.ROTATE_180) # rotate image by 180 since it's upside-down
        cv2_img = cv2.flip(cv2_img,1) # flip image horizontally
    except CvBridgeError, e:
        print(e)
    else:
        cv2.imwrite("CameraCalibration/cameraCal"+str(imageCounter)+".png", cv2_img) # save image
        print("Image saved!")

if __name__ == '__main__':
    main()