# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
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
    # TO DO: undistort image from calibration

    # TO DO: fix code below because it isn't segmenting colours properly

    # Apply Gaussian blur filter to image a bit
    img = cv2.GaussianBlur(img, (3,3), 0) 

    # convert to RGB (not working for some reason?)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # img = cv2.Sobel(src=img, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=3) # attempt sobel algorithm for edge detection

    plt.imshow(img)
    plt.show()
    
    # convert to float
    img = np.float32(img)

    # define stopping criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)

    # Use k means clustering to segment image into black and red
    k = 3
    _, labels, (centers) = cv2.kmeans(img, k, None, criteria, 5, cv2.KMEANS_RANDOM_CENTERS)

    # convert back to 8 bit values
    centers = np.uint8(centers)

    # flatten the labels array
    labels = labels.flatten()

    # convert all pixels to the color of the centroids
    segImg = centers[labels.flatten()]
    
    # reshape back to the original image dimension
    segImg = segImg.reshape(img.shape)

    # show the segmented image
    plt.imshow(segImg)
    plt.show()

    x = 0
    y = 0
    return x,y

if __name__ == '__main__':
    main()