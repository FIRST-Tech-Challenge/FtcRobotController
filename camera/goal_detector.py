from imutils.video import VideoStream
import numpy as np
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

vs = VideoStream(src=0).start()
time.sleep(2.0)
while True:
        frame = vs.read()
        frame = imutils.resize(frame, width=600, height=600)

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, Lower, Upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0:

            c = max(cnts, key=cv2.contourArea)
            ((x, y), size) = cv2.minEnclosingCircle(c)
            print(x,y)

            start_x = (int(x),0)
            end_x = (int(x),600)
            start_y = (0,int(y))
            end_y = (600,int(y))
            cv2.line(frame, start_x, end_x, (255,0,0), 2)
            cv2.line(frame, start_y, end_y, (255,0,0), 2)

        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
vs.stop()
cv2.destroyAllWindows()
