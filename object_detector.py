# Import required Libraries
from tkinter import *
from PIL import Image, ImageTk
import cv2
import numpy as np
import threading
# from tracker import *
import time

# BGR upper and lower bounds for detection (0-255)
lowB = 0
lowG = 0
lowR = 100
highB = 100
highG = 80
highR = 255

# Array of center positions
cx = np.array([])
cy = np.array([])

# Text
data1_text = ''
data2_text = ''
data3_text = ''
data4_text = ''
data5_text = ''
data6_text = ''
imageLabel = ''
outLabel = ''

# frame rescaling percentage
frame_scale = 0.5

def main(cap):
    global lowB
    global lowG
    global lowR

    global highB
    global highG 
    global highR 

    global cx
    global cy

    global data1_text
    global data2_text
    global data3_text
    global data4_text
    global data5_text
    global data6_text
    global outLabel
    global imageLabel

    global frame_scale

    # -----------------USER INPUT-----------------
    # filepath to photo for analysis
    # path_detectimage = './visionDebuggerImages/sampleImage1.png'
    #---------------------------------------------

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

    # BGR Inputs:
    data1_text = StringVar()
    data2_text = StringVar()
    data3_text = StringVar()
    data4_text = StringVar()
    data5_text = StringVar()
    data6_text = StringVar()

    data1_entry = Entry(textvariable = data1_text, width = 5)
    data2_entry = Entry(textvariable = data2_text, width = 5)
    data3_entry = Entry(textvariable = data3_text, width = 5)
    data4_entry = Entry(textvariable = data4_text, width = 5)
    data5_entry = Entry(textvariable = data5_text, width = 5)
    data6_entry = Entry(textvariable = data6_text, width = 5)

    data1_entry.place(x = 10, y = 40)
    data2_entry.place(x = 40, y = 40)
    data3_entry.place(x = 70, y = 40)
    data4_entry.place(x = 120, y = 40)
    data5_entry.place(x = 150, y = 40)
    data6_entry.place(x = 180, y = 40)

    data1_text.set(lowB)
    data2_text.set(lowG)
    data3_text.set(lowR)
    data4_text.set(highB)
    data5_text.set(highG)
    data6_text.set(highR)

    # cap = cv2.imread(path_detectimage)
    # cap = cv2.VideoCapture(0)

    # Repeat after an interval to capture continiously
    # threading.Thread(target = processCV(cap)).start()
    # win.mainloop()

    processCV(cap)

    return cx,cy

def rescaleFrame(frame, scale):
    """ Rescales the frame of the OpenCV image
    """
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)

    dimensions = (width, height)

    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

def readBGRBoxes():
    global lowB
    global lowG
    global lowR

    global highB
    global highG 
    global highR 
        
    if(data1_text.get() != ''):
        lowB = int(data1_text.get())
    else:
        lowB = 0

    if(data2_text.get() != ''):
        lowG = int(data2_text.get())
    else:
        lowG = 0
    
    if(data3_text.get() != ''):
        lowR = int(data3_text.get())
    else:
        lowR = 0

    if(data4_text.get() != ''):
        highB = int(data4_text.get())
    else:
        highB = 0

    if(data5_text.get() != ''):
        highG = int(data5_text.get())
    else:
        highG = 0

    if(data6_text.get() != ''):
        highR = int(data6_text.get())
    else:
        highR = 0

# Define function to show frame
def processCV(cap):
    global lowB
    global lowG
    global lowR

    global highB
    global highG 
    global highR 

    global cx
    global cy

    global data1_text
    global data2_text
    global data3_text
    global data4_text
    global data5_text
    global data6_text
    global outLabel
    global imageLabel

    global frame_scale

    # while(True):
    # Get the latest frame and convert into Image
    # avail, frame = cap.read()
    avail = True # Can remove all avails
    frame = rescaleFrame(cap,1)
    readBGRBoxes()
    cv2.imshow('img',frame)
    cv2.waitKey(1000)
    # Do all image transforms before here

    if(avail != False):
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
        # frame = rescaleFrame(frame,frame_scale)
        
        # Convert to RBG Image
        cv2image = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        cv2out = cv2.cvtColor(output,cv2.COLOR_BGR2RGB)
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

        cv2.imshow('img',frame)
        cv2.waitKey(2000)
    
if __name__ == '__main__':
    main(cv2.imread('./visionDebuggerImages/sampleImage1.png'))