import numpy as np
import cv2
import glob
import json

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = {} # 3d point in real world space
imgpoints = {} # 2d points in image plane.

# calibrate stereo

counter = 0
images = glob.glob('imgs/*.jpg')
images.sort()
objpoints = []
imgpoints = []
for fname in images:
    img = cv2.imread(fname)
    img=cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # cv2.imshow('winname', gray)
    # cv2.waitKey(0)  

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)
        counter += 1
        chess = cv2.drawChessboardCorners(img, (8,6), corners,ret)
        # cv2.imshow('winname', chess)
        # # cv2.imwrite('chess3/chess'+str(counter)+side+fname.split('/')[1],chess)
        # cv2.waitKey(0)    
    else:
        print(fname)

assert counter == len(images), "missed chessboard!!"
#exit(0)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

print(ret,mtx,dist,rvecs,tvecs)

np.savez('calib_map',ret=ret,mtx=mtx,dist=dist,rvecs=rvecs,tvecs=tvecs)