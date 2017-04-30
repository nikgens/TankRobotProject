from __future__ import division
import cv2
import numpy as np
import argparse
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from imutils.perspective import four_point_transform
# libraries to send data to Serial port
import serial
import struct

# default values for servos
currentPan = 95
currentTilt = 45

# default values for PID
MAX_MOTOR_SPEED = 230 # from 127 to 255
e_prev = 0 #error previous
e_int = 0 #integral error

# start values for HSV range, that can be choosen with findHSVRange() on startup
v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = 0, 0, 0, 180, 255, 255

# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
x_start, y_start, x_end, y_end = 0, 0, 0, 0
cropping = False
getROI = False
refPt = []
lower = np.array([])
upper = np.array([])

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

# record video from the camera
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 6.0, (640,480))

# parameters of the center of the frame
halfFrameWidth = cameraResolution[0]/2
halfFrameHeight = cameraResolution[1]/2

#define serial port
usbport = '/dev/ttyUSB0'
serialArduino = serial.Serial(usbport, 9600, timeout=1)

###########################
# Block of help functions #
###########################

def getArguments():
    '''
    Help function to hold script arguments
    '''
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--programm', required=True,
                    help='Specify programm to start: "-p line" - line following, "-p sign" - move with signs, "-p track" - color object tracking')
    ap.add_argument('-c', '--color', required=False,
                    help='Start HSV trackbar to choose color',
                    action='store_true')
    args = vars(ap.parse_args())
    return args


def mapValueToRange(value, fromMin, fromMax, toMin, toMax):
    '''
    Mapping function from one range to another
        >>> translate(127, 0, 255, -255, 255) #translate value 127 from range [0, 255] to range [-255,255]
    '''
    # Figure out how 'wide' each range is
    fromSpan = fromMax - fromMin
    toSpan = toMax - toMin
    
    # Convert the left range into a 0-1 range (float)
    valueScaled = (value - fromMin) / fromSpan
    
    # Convert the 0-1 range into a value in the right range.
    return int(toMin + (valueScaled * toSpan))


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


def pidController(xCentroidCoordinate, xCenterOfTheImage, Kp, Kd, Ki):

    global e_prev
    global e_int
    error = xCentroidCoordinate - xCenterOfTheImage
    e_int = e_int + error
    e_diff = error - e_prev
    pid = Kp * error + Ki * e_int + Kd * e_diff
    #print('pid (%d)= (Kp(%d) * error(%d))%d + (Ki(%d) * e_int(%d))%d + (Kd(%d) * e_diff(%d))%d' % (pid, Kp, error, Kp*error, Ki, e_int, Ki*e_int, Kd, e_diff, Kd*e_diff ))
    e_prev = error
    if abs(pid) < MAX_MOTOR_SPEED:
        return pid
    else:
        if pid > MAX_MOTOR_SPEED:
            return MAX_MOTOR_SPEED
        elif pid < -MAX_MOTOR_SPEED:
            return -MAX_MOTOR_SPEED
    

def movePanTilt(servo, angle):
    '''Moves the specified servo to the supplied angle.

    Arguments:
        servo
          the servo number to command, an integer from 1-4
        angle
          the desired servo angle, an integer from 0 to 180

    (e.g.) >>> servo.move(2, 90)
           ... # "move servo #2 to 90 degrees"'''

    if (0 <= angle <= 180):
        serialArduino.write(struct.pack('>B', 255)) # code 255 use for arduino for servo angles
        serialArduino.write(struct.pack('>B', servo))
        serialArduino.write(struct.pack('>B', angle))
        if servo == 1:
            print("Pan angle is: ", angle)
        else:
            print("Tilt angle is: ", angle)
    else:
        print ("Servo angle must be an integer between 0 and 180.\n")


def moveMotors(left, right):
    '''Moves the motors with specific speed.
       We can send to serial only values between 0 and 255.
       To move motors in different directions we define 0-126 for back motion
       and 128-255 for stright motion.
       On Arduino we will map values like this leftPWM = map(leftPWM, 0, 255, -255, 255);
       127 will be for stop the motors
    '''

    if (0 <= left <= 255) and (0 <= right <= 255):
        serialArduino.write(struct.pack('>B', 254)) # code 254 for move() function on Arduino
        serialArduino.write(struct.pack('>B', left))
        serialArduino.write(struct.pack('>B', right))

    else:
        print ("Speed must be an integer between 0 and 255.\n")


def calculateAnglesToMove(coordinates):
    ''' The function takes the coordinates of the centroid of the largest object that was substructed from the image
        and calculates new coordinates of pan/tilt servos to destinct the center of this object with
        the camera
    '''
    global currentPan
    global currentTilt
    
    # calculate difference in pixels between center of the frame and centroid coordinates
    differenceInX = halfFrameWidth - coordinates[0]
    differenceInY = halfFrameHeight - coordinates[1]
    
    # calculate angle that must be add/subtract to/from current servos position to reach
    # the center of the frame with centroid. 6 pix is the approximate value for 1 degree servo movement for (320, 240) frame
    if camera.resolution == (640, 480):
        changePanSeroAngleBy = differenceInX/12
        changeTiltSeroAngleBy = differenceInY/12
    elif camera.resolution == (320, 240):
        changePanSeroAngleBy = differenceInX/6
        changeTiltSeroAngleBy = differenceInY/6

    if changePanSeroAngleBy > 0:
        currentPan += abs(changePanSeroAngleBy)
    else:
        currentPan -= abs(changePanSeroAngleBy)

    if currentPan > 180:
        currentPan = 180
    elif currentPan < 0:
        currentPan = 0
        
    panAngle = currentPan
    #print ("currentPan: %d" % currentPan)

    if changeTiltSeroAngleBy > 0:
        currentTilt += abs(changeTiltSeroAngleBy)
    else:
        currentTilt -= abs(changeTiltSeroAngleBy)

    if currentTilt > 180:
        currentTilt = 180
    elif currentTilt < 0:
        currentTilt = 0
    
    tiltAngle = currentTilt
    #print ("currentTilt: %d" % currentTilt)

    return panAngle, tiltAngle


def identifyTrafficSign(image):
    '''
    In this function we select some ROI in which we expect to have the sign parts. If the ROI has more active pixels than threshold we mark it as 1, else 0
    After path through all four regions, we compare the tuple of ones and zeros with keys in dictionary SIGNS_LOOKUP
    It's the help function for followTheTrafficSigns()
    '''

    # define the dictionary of signs segments so we can identify
    # each signs on the image
    SIGNS_LOOKUP = {
        (1, 0, 0, 1): 'Turn Right', # turnRight
        (0, 0, 1, 1): 'Turn Left', # turnLeft
        (0, 1, 0, 1): 'Move Straight', # moveStraight
        (1, 0, 1, 1): 'Turn Back', # turnBack
    }

    THRESHOLD = 150
    
    image = cv2.bitwise_not(image)
    # (roiH, roiW) = roi.shape
    #subHeight = thresh.shape[0]/10
    #subWidth = thresh.shape[1]/10
    (subHeight, subWidth) = np.divide(image.shape, 10)
    subHeight = int(subHeight)
    subWidth = int(subWidth)

    # mark the ROIs borders on the image
    #cv2.rectangle(image, (subWidth, 4*subHeight), (3*subWidth, 9*subHeight), (0,255,0),2) # left block
    #cv2.rectangle(image, (4*subWidth, 4*subHeight), (6*subWidth, 9*subHeight), (0,255,0),2) # center block
    #cv2.rectangle(image, (7*subWidth, 4*subHeight), (9*subWidth, 9*subHeight), (0,255,0),2) # right block
    #cv2.rectangle(image, (3*subWidth, 2*subHeight), (7*subWidth, 4*subHeight), (0,255,0),2) # top block

    # substract 4 ROI of the sign thresh image
    leftBlock = image[4*subHeight:9*subHeight, subWidth:3*subWidth]
    centerBlock = image[4*subHeight:9*subHeight, 4*subWidth:6*subWidth]
    rightBlock = image[4*subHeight:9*subHeight, 7*subWidth:9*subWidth]
    topBlock = image[2*subHeight:4*subHeight, 3*subWidth:7*subWidth]

    # we now track the fraction of each ROI
    leftFraction = np.sum(leftBlock)/(leftBlock.shape[0]*leftBlock.shape[1])
    centerFraction = np.sum(centerBlock)/(centerBlock.shape[0]*centerBlock.shape[1])
    rightFraction = np.sum(rightBlock)/(rightBlock.shape[0]*rightBlock.shape[1])
    topFraction = np.sum(topBlock)/(topBlock.shape[0]*topBlock.shape[1])

    segments = (leftFraction, centerFraction, rightFraction, topFraction)
    segments = tuple(1 if segment > THRESHOLD else 0 for segment in segments)

    cv2.imshow("Warped", image)

    if segments in SIGNS_LOOKUP:
        return SIGNS_LOOKUP[segments]
    else:
        return None


def click_and_crop(event, x, y, flags, param):
    '''
    MouseCallback function
    '''
    # grab references to the global variables
    global x_start, y_start, x_end, y_end, cropping, getROI

    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        x_start, y_start, x_end, y_end = x, y, x, y
        cropping = True

    elif event == cv2.EVENT_MOUSEMOVE:
        if cropping == True:
            x_end, y_end = x, y

    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        x_end, y_end = x, y
        cropping = False
        getROI = True


def getColorFromCrop():
    global getROI, lower, upper
    done = False
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", click_and_crop)
        
    while True:
        # grab the current frame
        camera.capture(rawCapture, use_video_port=True, format='bgr')
        frame = rawCapture.array
        
        if not getROI:

            if not cropping:
                cv2.imshow("image", frame)

            elif cropping:
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
                cv2.imshow("image", frame)

        elif getROI and not done:
            cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
            cv2.imshow("image", frame)


            # if there are two reference points, then crop the region of interest
            # from teh image and display it
            refPt = [(x_start, y_start), (x_end, y_end)]

            roi = frame[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]

            hsvRoi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            print('min H = {}, min S = {}, min V = {}; max H = {}, max S = {}, max V = {}'.format(hsvRoi[:,:,0].min(), hsvRoi[:,:,1].min(), hsvRoi[:,:,2].min(), hsvRoi[:,:,0].max(), hsvRoi[:,:,1].max(), hsvRoi[:,:,2].max()))
         
            lower = np.array([hsvRoi[:,:,0].min(), hsvRoi[:,:,1].min(), hsvRoi[:,:,2].min()])
            upper = np.array([hsvRoi[:,:,0].max(), hsvRoi[:,:,1].max(), hsvRoi[:,:,2].max()])
            done = True


        if done:

            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask for the color from dictionary`1, then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            kernel = np.ones((9,9),np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                    
            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
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
            
                # only proceed if the radius meets a minimum size. Correct this value for your obect's size
                if radius > 0.5:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.putText(frame, 'center: {}, {}'.format(int(x), int(y)), (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

                 
            # show the frame to our screen
            cv2.imshow("image", frame)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
        elif key == ord("r"):
            getROI = False
     
              

###########################
# End of help functions   #
###########################

def trackTheColorObject():

    if args['color']:
        lower = np.array([v1_min, v2_min, v3_min])
        upper = np.array([v1_max, v2_max, v3_max])
    else:
        # define the lower and upper boundaries of the "orange"
        # ball in the HSV color space, then initialize the
        # list of tracked points
        lower = np.array([55,100,0])
        upper = np.array([75,255,255])
    
    while True:
        #start = time.time()

        # The use_video_port parameter controls whether the camera's image or video port is used 
        # to capture images. It defaults to False which means that the camera's image port is used. 
        # This port is slow but produces better quality pictures. 
        # If you need rapid capture up to the rate of video frames, set this to True.
        camera.capture(rawCapture, use_video_port=True, format='bgr')

        # At this point the image is available as stream.array
        frame = rawCapture.array
       
        # Draw the center of the image
        cv2.line(frame,(halfFrameWidth - 20,halfFrameHeight),(halfFrameWidth + 20,halfFrameHeight),(0,255,0),2)
        cv2.line(frame,(halfFrameWidth,halfFrameHeight - 20),(halfFrameWidth,halfFrameHeight + 20),(0,255,0),2)

        frame_to_thresh = frame.copy()
        hsv = cv2.cvtColor(frame_to_thresh, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
     
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw the center of the tracking object
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # draw the line from the center of the frame to the object's center
            cv2.line(frame,(halfFrameWidth,halfFrameHeight),(center),(255,0,0),2)

            # calculate new pan/tilt angle
            panAngle, tiltAngle = calculateAnglesToMove(center)

            # move servos. send command to the arduino
            movePanTilt(1,panAngle)
            movePanTilt(2,tiltAngle)
            time.sleep(0.2)

        # show images
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)

        out.write(frame)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            out.release()
            moveMotors(127,127)
            movePanTilt(1,currentPan)
            movePanTilt(2,currentTilt)
            print("Stop programm and close all windows")
            break
        #stop = time.time()
        #print(stop-start)


def followTheLine():
    if args['color']:
        lower = np.array([v1_min, v2_min, v3_min])
        upper = np.array([v1_max, v2_max, v3_max])
    else:
        # define the lower and upper boundaries of the
        # red line in the HSV color space, then initialize the
        # list of tracked points
        lower1 = np.array([0,100,80])
        upper1 = np.array([10,255,255])
        lower2 = np.array([170,100,80])
        upper2 = np.array([180,255,255])

    # move servos. send command to the arduino
    movePanTilt(1,currentPan)
    movePanTilt(2,0) #0 is the lowest
    time.sleep(0.2)

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
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 + mask2
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # cut ROI to minimize the centroid error with large line
        mask[0:90, 0:640] = 0
        mask[320:, 0:640] = 0

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
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw the center of the tracking object
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # if there is line find calculate the pid value to move along the line
            pid = pidController(center[0], halfFrameWidth, 0.5, 0.19, 0.04) #0.5, 0.192, 0.03
            # if pid is negative then down the speed of the left motor for that value and slow a little right motor
            if pid < 0:
                moveMotors(MAX_MOTOR_SPEED + pid, MAX_MOTOR_SPEED + pid*0.1)
            # vise versa for positive pid
            else:
                moveMotors(MAX_MOTOR_SPEED - pid*0.1, MAX_MOTOR_SPEED - pid)
        # stop the motors if there is now contour find    
        else:
            moveMotors(127,127)
            
        # show images
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        # write video
        out.write(frame)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            out.release()
            moveMotors(127,127)
            movePanTilt(1,currentPan)
            movePanTilt(2,currentTilt)
            print("Stop programm and close all windows")
            break


def followTheTrafficSigns():
    '''
    This function find blobs with blue color on the image.
    After blobs were found it detects the largest square blob, that must be the sign.
    '''
    # move servos to start position. send command to the arduino
    movePanTilt(1,currentPan)
    movePanTilt(2,currentTilt)
    time.sleep(2)

    lastDetectedTrafficSign = None
    
    if args['color']:
        lower = np.array([v1_min, v2_min, v3_min])
        upper = np.array([v1_max, v2_max, v3_max])
    else:
        # define range HSV for blue color of the traffic sign
        lower = np.array([80,0,0])
        upper = np.array([130,255,255])

    while True:
        # The use_video_port parameter controls whether the camera's image or video port is used 
        # to capture images. It defaults to False which means that the camera's image port is used. 
        # This port is slow but produces better quality pictures. 
        # If you need rapid capture up to the rate of video frames, set this to True.
        camera.capture(rawCapture, use_video_port=True, format='bgr')

        # At this point the image is available as stream.array
        frame = rawCapture.array
        frame_to_thresh = frame.copy()

        frameArea = frame.shape[0]*frame.shape[1]
        
        # convert color image to HSV color scheme
        hsv = cv2.cvtColor(frame_to_thresh, cv2.COLOR_BGR2HSV)
        
        # define kernel for smoothing   
        kernel = np.ones((3,3),np.uint8)
        # extract binary image with active blue regions
        mask = cv2.inRange(hsv, lower, upper)
        # morphological operations
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        # defite string variable to hold detected sign description
        detectedTrafficSign = None
        
        # define variables to hold values during loop
        largestArea = 0
        largestRect = None
        largestContour = None
        center = None
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            for cnt in cnts:
                # Rotated Rectangle. Here, bounding rectangle is drawn with minimum area,
                # so it considers the rotation also. The function used is cv2.minAreaRect().
                # It returns a Box2D structure which contains following detals -
                # ( center (x,y), (width, height), angle of rotation ).
                # But to draw this rectangle, we need 4 corners of the rectangle.
                # It is obtained by the function cv2.boxPoints()
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                # count euclidian distance for each side of the rectangle
                sideOne = np.linalg.norm(box[0]-box[1])
                sideTwo = np.linalg.norm(box[0]-box[3])
                # count area of the rectangle
                area = sideOne*sideTwo
                # find the largest rectangle within all contours
                if area > largestArea:
                    largestArea = area
                    largestRect = box
                    largestContour = cnt

        #print("Largest Area: %d, Frame Area: %d" % (largestArea, frameArea))    
        if largestArea > frameArea*0.001:
                   
            # find moment for the rectangle
            M = cv2.moments(largestContour)
            # find center of the rectangle
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if largestArea > frameArea*0.15:
                moveMotors(127,127) #127 is mapped to 0 on Arduino
                print("Big sign to close")
                time.sleep(0.5)
                if lastDetectedTrafficSign == 'Turn Right':
                    #right
                    moveMotors(255,0)
                    time.sleep(0.39)
                    moveMotors(127,127)
                    print("Turned to the right")
                    time.sleep(0.5)
                elif lastDetectedTrafficSign == 'Turn Left':
                    #left
                    moveMotors(0,255)
                    time.sleep(0.35)
                    moveMotors(127,127)
                    print("Turned to the left")
                    time.sleep(0.5)
                elif lastDetectedTrafficSign == 'Turn Back':
                    #reverse_right
                    moveMotors(255,0)
                    time.sleep(0.73)
                    moveMotors(127,127)
                    print("Turned back")
                    time.sleep(0.5)
                elif lastDetectedTrafficSign == 'Move Straight':
                    print("Go, go, go")

            else:
                # count error with PID to move to the sign direction
                pid = pidController(center[0], halfFrameWidth, 0.5, 0, 0)
 
                # if error with "-", then we need to slow down left motor, else - right
                if pid < 0:
                    moveMotors(MAX_MOTOR_SPEED + pid, MAX_MOTOR_SPEED + pid*0.1)
                else:
                    moveMotors(MAX_MOTOR_SPEED - pid*0.1, MAX_MOTOR_SPEED - pid)
            
            # draw contour of the found rectangle on  the original image   
            cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
            
            # cut and warp interesting area
            warped = four_point_transform(mask, [largestRect][0])
            
            # show an image if rectangle was found
            cv2.imshow("Warped", cv2.bitwise_not(warped))
			
	    # use function to detect the sign on the found rectangle
            detectedTrafficSign = identifyTrafficSign(warped)
            print('detectedTrafficSign', detectedTrafficSign)
            if detectedTrafficSign is not None:
                lastDetectedTrafficSign = detectedTrafficSign
            print('lastDetectedTrafficSign', lastDetectedTrafficSign)
	    # write the description of the sign on the original image
            cv2.putText(frame, detectedTrafficSign, tuple(largestRect[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        
        # if there is no blue rectangular on the frame, then stop
        else:
            moveMotors(127,127) #127 is mapped to 0 on Arduino
            print("No sign")
            
        # show original image
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)

        out.write(frame)
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            out.release()
            moveMotors(127,127)
            movePanTilt(1,currentPan)
            movePanTilt(2,currentTilt)
            print("Stop programm and close all windows")
            break


def moveToTheObject():

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
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
     
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            objectArea = M["m00"]
            frameArea = cameraResolution[0]*cameraResolution[1]
            percentage = objectArea/frameArea*100
            
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw the center of the tracking object
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            if percentage < 5:
                moveMotors(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)
                print ("Object is too far, move to the object. Percentage is {}".format(percentage))
            elif percentage > 10:
                moveMotors(255 - MAX_MOTOR_SPEED, 255 - MAX_MOTOR_SPEED)
                print ("Object is too close, move from the object. Percentage is {}".format(percentage))
            
            # if there is object find calculate the pid value to move along the line
            pid = pidController(center[0], halfFrameWidth, 0.34, 0, 0)
            if pid > 127:
                pid = 127
            elif pid < -127:
                pid = -127
            elif 0 > pid > -60:
                pid = 0
            elif 0 < pid < 60:
                pid = 0

            moveMotors(127 + pid, 127 - pid)
            print('PID is %d. moveMotors(%d, %d)' % (pid, 127 + pid, 127 - pid))

        # stop the motors if there is now contour find    
        else:
            moveMotors(127,127)
            
        # show images
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        # write video
        out.write(frame)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            out.release()
            moveMotors(127,127)
            print("Stop programm and close all windows")
            break
    

def main():
    if args['programm'] == 'sign':
        if args['color']:
            findHSVRange()
        followTheTrafficSigns()
    elif args['programm'] == 'track':
        if args['color']:
            findHSVRange()
        trackTheColorObject()
    elif args['programm'] == 'line':
        if args['color']:
            movePanTilt(1,currentPan)
            movePanTilt(2,0)
            time.sleep(0.2)
            findHSVRange()
        followTheLine()
    elif args['programm'] == 'follow':
        if args['color']:
            getColorFromCrop()
            #findHSVRange()
        moveToTheObject()
        
        
if __name__ == '__main__':
    # to store args as global variable. If to set this line of code in main() we will not have the opportunity
    # to use args values in other functions
    args = getArguments()
    # start the main function
    main()
