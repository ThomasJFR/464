# Import required Libraries
from tkinter import *
from PIL import Image, ImageTk
import cv2
import numpy as np
import time

# BGR upper and lower bounds for detection (0-255)
lowB = 0
lowG = 0
lowR = 100
highB = 100
highG = 100
highR = 255

# Array of center positions
cx = np.array([])
cy = np.array([])

# Text
imageLabel = ''
outLabel = ''

# frame rescaling percentage
frame_scale = 1 # already scaled in camera calibration

def main(cap):
    global lowB
    global lowG
    global lowR

    global highB
    global highG 
    global highR 

    global cx
    global cy

    global outLabel
    global imageLabel

    global frame_scale

    # Create an instance of TKinter Window or frame
    win= Tk()

    # Set the size of the window
    win.geometry("1000x500")# Create a Label to capture the Video frames

    imageLabel = Label(win)
    imageLabel.place(x = 0, y = 80)

    outLabel = Label(win)
    outLabel.place(x = 500, y = 80)

    ballPosLabel = Label(text = "Ball Position",bd =4)
    ballPosLabel.place(x = 250, y = 10)

    processCV(cap)

    return cx,cy

# Define function to scale image
def rescaleFrame(frame, scale):
    """ Rescales the frame of the OpenCV image
    """
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA), width, height

# Define function to show frame and identify object centres
def processCV(cap):
    global lowB
    global lowG
    global lowR

    global highB
    global highG 
    global highR 

    global cx
    global cy

    global outLabel
    global imageLabel

    global frame_scale

    frame, frame_w, frame_h = rescaleFrame(cap,frame_scale)
    cv2.imshow('img',frame)
    cv2.waitKey(1000)

    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    # Lower bound and upper bound for red
    lower_bound = np.array([lowB, lowG, lowR])     
    upper_bound = np.array([highB, highG, highR])

    # Create mask based on bounds
    mask = cv2.inRange(frame, lower_bound, upper_bound)
    output = cv2.bitwise_and(frame, frame, mask=mask)

    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the size and position of the largest contour
    minContour = 2 # Adjust at will
    i = 0
    for cnt in contours:
        cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
        area = cv2.contourArea(cnt)
        if area > minContour:
            x, y, w, h = cv2.boundingRect(cnt)
            # Draw Center and Bounding Box of the contour
            cxi = int((x + x + w)/2)
            cyi = int((y + y + h)/2)
            cx = np.append(cx,cxi)
            cy = np.append(cy,cyi)
            cv2.circle(frame, (cxi, cyi), 3, (255,0,0), -1) 
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
            i += 1
    i -= 1

    cx = cx - frame_w/2 # move origin from centre of image to top right corner
    cy = cy - frame_h/2 # move origin from centre of image to top right corner

    # Convert to RGB Image
    cv2image = frame#cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    cv2out = output#cv2.cvtColor(output,cv2.COLOR_BGR2RGB)
    img = Image.fromarray(cv2image)
    out = Image.fromarray(cv2out)

    # Convert Image to PhotoImage
    imgtk = ImageTk.PhotoImage(image = img)
    imageLabel.imgtk = imgtk
    imageLabel.configure(image=imgtk)

    outtk = ImageTk.PhotoImage(image = out)
    outLabel.mastk = outtk
    outLabel.configure(image=outtk)

    # ballPosLabel['text'] = str(cx[i])+","+str(cy[i])
    time.sleep(0.014) # Bug from Trevors implementation, only needed for live feed

    cv2.imshow('Object detected',frame)
    cv2.waitKey(2000)
    
if __name__ == '__main__':
    main(cv2.imread('./visionDebuggerImages/sampleImage1.png'))