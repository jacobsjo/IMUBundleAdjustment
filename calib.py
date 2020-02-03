import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('data/chess6_7/*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()

#camera matrix, distortion coefficients, rotation and translation vectors
#dist  =[k1,k2,p1,p2,k3]
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#Convert rvecs, tvecs to list of 3 elements

print (rvecs[0].shape)
print (tvecs[0].shape)
with open('data/calib.txt',"ab") as f:
    np.savetxt(f,mtx)
    f.write(b"\n")
    np.savetxt(f,dist)
    f.write(b"\n")
    np.savetxt(f,rvecs[0])
    f.write(b"\n")
    np.savetxt(f,tvecs[0])

print ('ret',ret)
print ('camera matrix', mtx)
print ('distortion coefficients',dist)
print ('rotation vectors',rvecs)
print ('translation vectors',tvecs)

#TODO:
#
