import numpy as np
import cv2 as cv
import glob

chessboardSize = (9, 6)
frameSize = (1280,720)

criteria = (cv.TERM_CRITERIA_EPS + cv.TermCriteria_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1,2)

objPoints = []
imgPoints = []

images = glob.glob('*.jpg')

for image in images:
    print(image)
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)
    
    if ret == True:
        objPoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgPoints.append(corners)

        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(1000)

cv.destroyAllWindows()

ret, cameraMatrix, dist, revecs, tvecs = cv.calibrateCamera(objPoints, imgPoints, frameSize, None, None)

print("Camera Calibrated: ", ret)
print("\nCamera Matrix:\n", cameraMatrix)
print("\nDistortion Parameters\n", dist)
print("\nRotation Vectors:\n", revecs)
print("\nTranslation Vectors\n", tvecs)
