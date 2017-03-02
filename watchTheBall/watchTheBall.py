import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.brightness = 60
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(2)

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
lower_yellow = np.array([40,80,90])
upper_yellow = np.array([60,255,255])

while True:
    camera.capture(rawCapture, format='bgr')
    # At this point the image is available as stream.array
    frame = rawCapture.array
    # Draw the center of the image
    cv2.line(frame,(frame.shape[1]/2 - 20,frame.shape[0]/2),(frame.shape[1]/2 + 20,frame.shape[0]/2),(0,255,0),2)
    cv2.line(frame,(frame.shape[1]/2,frame.shape[0]/2 - 20),(frame.shape[1]/2,frame.shape[0]/2 + 20),(0,255,0),2)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
 
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            print (int(x), int(y))

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)

    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        print("End Cam Tracking")
        break
    

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
