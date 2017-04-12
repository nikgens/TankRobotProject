from __future__ import division
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time


# start values for HSV range, that can be choosen with findHSVRange() on startup
v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = 0, 0, 0, 180, 255, 255

# initialize the camera and grab a reference to the raw camera capture
cameraResolution = (640, 480)
camera = PiCamera()
camera.resolution = cameraResolution
camera.framerate = 90
camera.brightness = 60
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=cameraResolution)
# allow the camera to warmup
time.sleep(2)


def findHSVRange():
    '''
    This is a help function to find HSV color ranges, that will be used in other functions of our robot
    It will cteate trackbars to find optimal range values from the captured images
    '''
    global v1_min, v2_min, v3_min, v1_max, v2_max, v3_max 

    # the function namedWindow creates a window that can be used as a placeholder for images and trackbars.
    # Created windows are referred to by their names.
    cv2.namedWindow("Trackbars", 0)
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255
        for j in 'HSV':
            if j == 'H':
                # create trackbar for Hue from 0 tj 180 degrees
                # For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]. 
                cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 179, (lambda x: None))
            else:
                cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, (lambda x: None))

    while True:

        camera.capture(rawCapture, use_video_port=True, format='bgr')
        frame = rawCapture.array
        frame_to_thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        values = []
        for i in ["MIN", "MAX"]:
            for j in 'HSV':
                v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
                values.append(v)

        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max =  values

        thresh = cv2.inRange(frame_to_thresh, np.array([v1_min, v2_min, v3_min]), np.array([v1_max, v2_max, v3_max]))
        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)

        rawCapture.truncate(0)
	
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            print("Stop findHSVRange() programm and close all windows")
            break

def moveToTheObject():
    lower = np.array([v1_min, v2_min, v3_min])
    upper = np.array([v1_max, v2_max, v3_max])

    while True:
        # The use_video_port parameter controls whether the camera's image or video port is used 
        # to capture images. It defaults to False which means that the camera's image port is used. 
        # This port is slow but produces better quality pictures. 
        # If you need rapid capture up to the rate of video frames, set this to True.
        camera.capture(rawCapture, use_video_port=True, format='bgr')

        # At this point the image is available as stream.array
        frame = rawCapture.array

        frame_to_thresh = frame.copy()
        hsv = cv2.cvtColor(frame_to_thresh, cv2.COLOR_BGR2HSV)
            
        kernel = np.ones((5,5),np.uint8)
        # for red color we need to masks.
        mask = cv2.inRange(hsv, lower, upper)
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
            M = cv2.moments(c)
            area = M["m00"]
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw the center of the tracking object
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            print('area: %d, frame area: %d, percentage: %d' %
                  (area, cameraResolution[1]*cameraResolution[0], area/(cameraResolution[1]*cameraResolution[0])*100))


        # show images
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            print("Stop programm and close all windows")
            break

def main():
    findHSVRange()
    moveToTheObject()
        
        
if __name__ == '__main__':
    main()
