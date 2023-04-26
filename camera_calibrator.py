import numpy as np
import cv2 as cv
import glob
import os

def main(imgInput):
    # -----------------USER INPUT-----------------
    # checkerboard size (count interior corners only, eg. a chessboard is 7x7 not 9x9)
    rows = 6
    columns = 7
    # image rescale percentage
    scale_percent = 100
    # calibration photos filepath
    calib_photos_path = os.path.join("./CameraCalibration", "cameraCal*.png")
    # filepath for photo to test undistortion (and calculate reproduction error)
    test_photo_path = './CameraCalibration/cameraCal7.png'
    # filepath for photo to calculate scaling factor
    scale_photo_path = './CameraCalibration/cameraCal7.png'
    # filepath for undistorted photo
    undist_photo_path = './CameraCalibration/calibresult.png'
    # size of each individual chessboard square in mm
    mm_per_square = 9 # mm
    #---------------------------------------------

    # -----------------MAIN CODE BODY-----------------
    images = glob.glob(calib_photos_path)

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:6,0:7].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob(os.path.join("./CameraCalibration", "*.png"))

    for fname in images:
        img = cv.imread(fname)

        # resize images
        # img = resize(img,scale_percent)

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # cv.imshow('gray',gray)
        # cv.waitKey(500)

        # Find the (interior) chess board corners
        ret, corners = cv.findChessboardCorners(gray, (rows,columns), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (rows,columns), corners2, ret)
            # cv.imshow('img', img)
            print('Success: ' + fname)
        else: 
            print('Failed to detect checkerboard: ' + fname)

    cv.destroyAllWindows()

    # Calibration
    # get camera matrix, distortion coeffs, rotation/translation vectors
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Undistort inupt image
    dst = undistort(resize(imgInput,scale_percent), mtx, dist)
    cv.imwrite(undist_photo_path, dst)   

    # Reprojection Error Calculation
    # 0 is best
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )

    # Undistort image for scaling factor
    dstScale = undistort(resize(cv.imread(scale_photo_path),scale_percent), mtx, dist)

    # Pixel to mm scaling factor (AFTER undistorting image!)
    print("Scaling Factor Computation")
    gray = cv.cvtColor(dstScale, cv.COLOR_BGR2GRAY)
    patternFound, corners = cv.findChessboardCorners(gray, (rows,columns), None)
    if(patternFound): # refines the corner location
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    # get distance between first two points
    pt1 = corners2[0,0,:]
    pt2 = corners2[1,0,:]
    dist1 = np.linalg.norm(pt1 - pt2)
    print(dist1)
    # get distance between last two points
    pt1 = corners2[-1,0,:]
    pt2 = corners2[-2,0,:]
    dist2 = np.linalg.norm(pt1 - pt2)
    print(dist2)
    # average the 2 distances and use that as num of pixels per chessboard square
    pixel_per_square = np.mean([dist1,dist2])
    print("pixel per square = " + str(pixel_per_square))
    pixel_per_mm = pixel_per_square/mm_per_square
    print("pixel per mm = " + str(pixel_per_mm))

    return dst,pixel_per_mm

def resize(img, scale_percent):
    # print('Original Dimensions : ',img.shape)
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    img = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    # print('Resized Dimensions : ',img.shape)
    return img

def undistort(img, mtx, dist):
    # img = cv.imread(test_photo_path)
    h,  w = img.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w] # the undistorted image
    
    return dst

if __name__ == '__main__':
    main(cv.imread('./visionDebuggerImages/sampleImage1.png'))

