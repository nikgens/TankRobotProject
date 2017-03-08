import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from imutils.perspective import four_point_transform
# send data to Serial port
import serial
import struct

currentPan = 90
currentTilt = 60

# default values for PID
maxMotorSpeed = 200
e_prev = 0
e_int = 0
Kp = 0.4
Kd = 0
Ki = 0

# initialize the camera and grab a reference to the raw camera capture
cameraResolution = (640, 480)
camera = PiCamera()
camera.resolution = cameraResolution
camera.framerate = 32
camera.brightness = 60
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=cameraResolution)

# record video from the camera
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 10.0, (640,480))

# parameters of center of the frame
halfFrameWidth = cameraResolution[0]/2
halfFrameHeight = cameraResolution[1]/2
 
# allow the camera to warmup
time.sleep(2)

#define serial port
usbport = '/dev/ttyUSB0'
serialArduino = serial.Serial(usbport, 9600, timeout=1)

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
        serialArduino.write(struct.pack('>B', 255)) # code 255 for servo angles
        serialArduino.write(struct.pack('>B', servo))
        serialArduino.write(struct.pack('>B', angle))
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

    if (0 <= left <= 255) or (0 <= right <= 255):
        serialArduino.write(struct.pack('>B', 254)) # code 254 for move() function on Arduino
        serialArduino.write(struct.pack('>B', left))
        serialArduino.write(struct.pack('>B', right))

    else:
        print ("Speed must be an integer between 0 and 255.\n")


def calculateAnglesToMove(coordinates):
    ''' The function takes the coordinates of the largest object that was substructed from the image
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


def followTheColoreObject():

    # define the lower and upper boundaries of the "orange"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    lower_yellow = np.array([0,100,100])
    upper_yellow = np.array([10,255,255])
    
    while True:
        start = time.time()

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

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)


        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            print("Stop programm and close all windows")
            break
        stop = time.time()
        print(stop-start)


def followTheLine():

    # define the lower and upper boundaries of the
    # red line in the HSV color space, then initialize the
    # list of tracked points
    lower_line = np.array([0,100,100])
    upper_line = np.array([10,255,255])

    # move servos. send command to the arduino
    movePanTilt(1,90)
    movePanTilt(2,25)
    time.sleep(0.2)

    while True:
        # The use_video_port parameter controls whether the camera's image or video port is used 
        # to capture images. It defaults to False which means that the camera's image port is used. 
        # This port is slow but produces better quality pictures. 
        # If you need rapid capture up to the rate of video frames, set this to True.
        camera.capture(rawCapture, use_video_port=True, format='bgr')

        # At this point the image is available as stream.array
        frame = rawCapture.array
       
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.inRange(hsv, lower_line, upper_line)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask[0:120, 0:320] = 0

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
            
            pid = pidController(center[0], halfFrameWidth)

            if pid > maxMotorSpeed:
                pid = maxMotorSpeed
            elif pid < -maxMotorSpeed:
                pid = -maxMotorSpeed

            if pid < 0:
                moveMotors(maxMotorSpeed + pid, maxMotorSpeed)
            else:
                moveMotors(maxMotorSpeed, maxMotorSpeed - pid)
            
        else:
            moveMotors(0,0)
            
        # show images
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)


        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            moveMotors(0,0)
            print("Stop programm and close all windows")
            break


def pidController(xCentroidCoordinate, xCenterOfTheImage):

    global e_prev
    global e_int
    error = xCentroidCoordinate - xCenterOfTheImage
    e_int = e_int + error
    e_diff = error - e_prev
    pid = Kp * error + Ki * e_int + Kd * e_diff
    #print('pid (%d)= Kp * error(%d)+ Ki * e_int(%d)+ Kd * e_diff(%d)' % (pid, error, e_int, e_diff))
    e_prev = error
    if abs(pid) < 128:
        return pid
    else:
        if pid > 128:
            return 128
        elif pid < -128:
            return -128
    


def findTrafficSign():
    '''
    This function find blobs with blue color on the image.
    After blobs were found it detects the largest square blob, that must be the sign.
    '''
	
    # define range HSV for blue color of the traffic sign
    lower_blue = np.array([80,80,40])
    upper_blue = np.array([110,255,255])

    while True:
        # The use_video_port parameter controls whether the camera's image or video port is used 
        # to capture images. It defaults to False which means that the camera's image port is used. 
        # This port is slow but produces better quality pictures. 
        # If you need rapid capture up to the rate of video frames, set this to True.
        camera.capture(rawCapture, use_video_port=True, format='bgr')

        # At this point the image is available as stream.array
        frame = rawCapture.array

        frameArea = frame.shape[0]*frame.shape[1]
        
        # convert color image to HSV color scheme
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # define kernel for smoothing   
        kernel = np.ones((3,3),np.uint8)
        # extract binary image with active blue regions
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
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
            
            # count error with PID to move to the sign direction
            pid = pidController(center[0], halfFrameWidth)
            # serialArduino.write(struct.pack('>B', 253)) from moveMotors() can only
            # transfer values from 0 to 255, so if the value of error greater then 255
            # we assign max value 255 and if less then 0, then we assign min value 0
            if pid > maxMotorSpeed:
                pid = maxMotorSpeed
            elif pid < -maxMotorSpeed:
                pid = -maxMotorSpeed
            # if error with "-", then we need to slow down left motor, else - right
            if pid < 0:
                moveMotors(maxMotorSpeed + pid, maxMotorSpeed)
            else:
                moveMotors(maxMotorSpeed, maxMotorSpeed - pid)
            
            # draw contour of the found rectangle on  the original image   
            cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
            
            # cut and warp interesting area
            warped = four_point_transform(mask, [largestRect][0])
            
            # show an image if rectangle was found
            cv2.imshow("Warped", cv2.bitwise_not(warped))
			
	    # use function to detect the sign on the found rectangle
            detectedTrafficSign = identifyTrafficSign(warped)
            #print(detectedTrafficSign)

	    # write the description of the sign on the original image
            cv2.putText(frame, detectedTrafficSign, tuple(largestRect[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        
        elif largestArea > frameArea*0.35:
            moveMotors(127,127) #127 is mapped to 0 on Arduino

        # if there is no blue rectangular on the frame, then stop
        else:
            moveMotors(127,127) #127 is mapped to 0 on Arduino
            
        # show original image
        cv2.imshow("Original", frame)

        out.write(frame)
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            out.release()
            print("Stop programm and close all windows")
            break

def identifyTrafficSign(image):
    '''
    In this function we select some ROI in which we expect to have the sign parts. If the ROI has more active pixels than threshold we mark it as 1, else 0
    After path through all four regions, we compare the tuple of ones and zeros with keys in dictionary SIGNS_LOOKUP
    It's the help function for findTrafficSign()
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


def main():
    findTrafficSign()


if __name__ == '__main__':
    main()
