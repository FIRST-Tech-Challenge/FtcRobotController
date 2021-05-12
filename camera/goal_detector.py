from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
# construct the argument parse and parse the arguments
# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
redLower = (0, 100, 100)
redUpper = (10, 255, 255)

orangeLower = (10,68,214)
orangeUpper = (38,252,255)

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

Upper = redUpper
Lower = redLower

# if a video path was not supplied, grab the reference
# to the webcam
vs = VideoStream(src=0).start()
# otherwise, grab a reference to the video file
# allow the camera or video file to warm up
time.sleep(2.0)
# keep looping
while True:
	# grab the current frame
        frame = vs.read()
        # handle the frame from VideoCapture or VideoStream
        
	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
        if frame is None:
            break
	# resize the frame, blur it, and convert it to the HSV
	# color space
        frame = imutils.resize(frame, width=600, height=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
        mask = cv2.inRange(hsv, Lower, Upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
	# (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:

            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
	    # only proceed if the radius meets a minimum size
            if radius > 10:
                print(x,y)
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                start_x = (int(x),0)
                end_x = (int(x),600)
                start_y = (0,int(y))
                end_y = (600,int(y))
                cv2.line(frame, start_x, end_x, (255,0,0), 2)
                cv2.line(frame, start_y, end_y, (255,0,0), 2)
	# update the points queue
	# # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
# if we are not using a video file, stop the camera video stream
# otherwise, release the camera
vs.stop()
# close all windows
cv2.destroyAllWindows()
