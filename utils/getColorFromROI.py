import numpy as np
import cv2
import functions

if functions.isRaspberry():
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import time
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 90
    camera.brightness = 60
    camera.rotation = 180
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    # allow the camera to warmup
    time.sleep(2)
else:
    camera = cv2.VideoCapture(0)


x_start, y_start, x_end, y_end = 0, 0, 0, 0
cropping = False
getROI = False



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
    lower = np.array([])
    upper = np.array([])
    refPt = []
    global getROI
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", click_and_crop)
        
    while True:

        if functions.isRaspberry():
            # grab the current frame
            camera.capture(rawCapture, use_video_port=True, format='bgr')
            frame = rawCapture.array
        else:
            ret, frame = camera.read()

            if ret == False:
                print('Failed to capture frame from camera. Check camera index in cv2.VideoCapture(0) \n')
                break
 
        if not getROI:

            if not cropping:
                cv2.imshow("image", frame)

            elif cropping:
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
                cv2.imshow("image", frame)

        else:
            cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
            cv2.imshow("image", frame)

            # if there are two reference points, then crop the region of interest
            # from teh image and display it
            refPt = [(x_start, y_start), (x_end, y_end)]

            roi = frame[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]

            hsvRoi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            lower = np.array([hsvRoi[:,:,0].min(), hsvRoi[:,:,1].min(), hsvRoi[:,:,2].min()])
            upper = np.array([hsvRoi[:,:,0].max(), hsvRoi[:,:,1].max(), hsvRoi[:,:,2].max()])
            if functions.isRaspberry():
                cv2.destroyAllWindows()
            else:
                camera.release()
            return lower, upper

        
        if functions.isRaspberry():
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            if functions.isRaspberry():
                cv2.destroyAllWindows()
            else:
                camera.release()
            break


if __name__ == '__main__':
    try:
        lower, upper = getColorFromCrop()
        print(lower, upper)
    except TypeError:
        print('Nothing was captured\n')
