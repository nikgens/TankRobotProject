import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from imutils.perspective import four_point_transform
#from imutils import contours
#import imutils

cameraResolution = (320, 240)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = cameraResolution
camera.framerate = 32
camera.brightness = 60
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=cameraResolution)

# allow the camera to warmup
time.sleep(2)

def findTrafficSign():
    '''
    This function find blobs with blue color on the image.
    After blobs were found it detects the largest square blob, that must be the sign.
    '''
	
    # define range HSV for blue color of the traffic sign
    lower_blue = np.array([90,80,50])
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
            
        if largestArea > frameArea*0.02:
            # draw contour of the found rectangle on  the original image   
            cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
            
            # cut and warp interesting area
            warped = four_point_transform(mask, [largestRect][0])
            
            # show an image if rectangle was found
            #cv2.imshow("Warped", cv2.bitwise_not(warped))
			
	    # use function to detect the sign on the found rectangle
            detectedTrafficSign = identifyTrafficSign(warped)
            #print(detectedTrafficSign)

	    # write the description of the sign on the original image
            cv2.putText(frame, detectedTrafficSign, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        
        # show original image
        cv2.imshow("Original", frame)
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            print("Stop programm and close all windows")
            break

def identifyTrafficSign(image):
    '''
    In this function we select some ROI in which we expect to have the sign parts. If the ROI has more active pixels than threshold we mark it as 1, else 0
    After path through all four regions, we compare the tuple of ones and zeros with keys in dictionary SIGNS_LOOKUP
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
