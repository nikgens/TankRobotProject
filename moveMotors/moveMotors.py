import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
# send data to Serial port
import serial
import struct

currentPan = 90
currentTilt = 60

maxMotorPWM = 255
e_prev = 0
e_int = 0
Kp = 1.8
Kd = 0.00001
Ki = 0.001

cameraResolution = (320, 240)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = cameraResolution
camera.framerate = 32
camera.brightness = 60
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=cameraResolution)

# parameters of center of the frame
halfFrameWidth = cameraResolution[0]/2
halfFrameHeight = cameraResolution[1]/2
 
# allow the camera to warmup
time.sleep(2)

# define the lower and upper boundaries of the "orange"
# ball in the HSV color space, then initialize the
# list of tracked points
lower_yellow = np.array([0,100,100])
upper_yellow = np.array([10,255,255])

# define the lower and upper boundaries of the
# red line in the HSV color space, then initialize the
# list of tracked points
lower_line = np.array([0,100,100])
upper_line = np.array([10,255,255])


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
    '''Moves the motors with specific speed'''

    if (0 < left <= 255) or (0 < right <= 255):
        serialArduino.write(struct.pack('>B', 254)) # code 254 for moveStright() function on Arduino
        serialArduino.write(struct.pack('>B', left))
        serialArduino.write(struct.pack('>B', right))
    elif left == 0 and right == 0:
        serialArduino.write(struct.pack('>B', 253)) # code 253 for stop() function on Arduino
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

            if pid > maxMotorPWM:
                pid = maxMotorPWM
            elif pid < -maxMotorPWM:
                pid = -maxMotorPWM

            if pid < 0:
                moveMotors(maxMotorPWM + pid, maxMotorPWM)
            else:
                moveMotors(maxMotorPWM, maxMotorPWM - pid)
            
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
    print('pid (%d)= Kp * error(%d)+ Ki * e_int(%d)+ Kd * e_diff(%d)' % (pid, error, e_int, e_diff))
    e_prev = error
    return pid


def main():
    followTheLine()


if __name__ == '__main__':
    main()
