import numpy as np
import cv2 as cv
import glob
import os

# -----------------USER INPUT-----------------
# checkerboard size (count interior corners only, eg. a chessboard is 7x7 not 9x9)
rows = 6
columns = 7

# image rescale percentage
scale_percent = 50

# calibration photos array
images = glob.glob(os.path.join("./calibration photos", "*.png"))

#---------------------------------------------

def resize(img, scale_percent):
    print('Original Dimensions : ',img.shape)
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    img = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    print('Resized Dimensions : ',img.shape)
    return img

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:7].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(os.path.join("./CameraCalibration", "*.png"))
scale_percent = 100

for fname in images:
    img = cv.imread(fname)

    # resize images
    img = resize(img,scale_percent)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    cv.imshow('gray',gray)
    cv.waitKey(500)

    # Find the (interior) chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,4), None)
    print(ret)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (rows,columns), corners2, ret)
        cv.imshow('img', img)

cv.destroyAllWindows()

# Calibration
# get camera matrix, distortion coeffs, rotation/translation vectors
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Undistortion
img = cv.imread('./CameraCalibration/cameraCal1.png')
img = resize(img,scale_percent)
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('./CameraCalibration/cameraCal1.png', dst)

# Reprojection Error Calculation
# 0 is best
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )