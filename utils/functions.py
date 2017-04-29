import cv2
import sys
import socket

e_prev, e_int = 0, 0



def isRaspberry():
    if socket.gethostbyaddr(socket.gethostname())[0] == 'raspberrypi':
        return True
    else:
        return False

def isOpenCV3():
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        return False
    else:
        return True

def isPython3():
    if sys.version_info[0] == 3:
        return True
    else:
        return False

def resize(img, width=None, height=None, interpolation = cv2.INTER_AREA):
    w, h, _ = img.shape
    
    if width is None and height is None:
        return img
    elif width is None:
        ratio = height/h
        width = int(w*ratio)
        print(width)
        resized = cv2.resize(img, (height, width), interpolation)
        return resized
    else:
        ratio = width/w
        height = int(h*ratio)
        print(height)
        resized = cv2.resize(img, (height, width), interpolation)
        return resized

def fourPointWarp(cnt, orig):
    # now that we have our screen contour, we need to determine
    # the top-left, top-right, bottom-right, and bottom-left
    # points so that we can later warp the image -- we'll start
    # by reshaping our contour to be our finals and initializing
    # our output rectangle in top-left, top-right, bottom-right,
    # and bottom-left order
    pts = cnt.reshape(4, 2)
    rect = np.zeros((4, 2), dtype = "float32")

    # summing the (x, y) coordinates together by specifying axis=1
    # the top-left point has the smallest sum whereas the
    # bottom-right has the largest sum
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    # compute the difference between the points -- the top-right
    # will have the minumum difference and the bottom-left will
    # have the maximum difference
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # Notice how our points are now stored in an imposed order: 
    # top-left, top-right, bottom-right, and bottom-left. 
    # Keeping a consistent order is important when we apply our perspective transformation


    # now that we have our rectangle of points, let's compute
    # the width of our new image
    (tl, tr, br, bl) = rect
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))

    # ...and now for the height of our new image
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))

    # take the maximum of the width and height values to reach
    # our final dimensions
    maxWidth = max(int(widthA), int(widthB))
    maxHeight = max(int(heightA), int(heightB))

    # construct our destination points which will be used to
    # map the screen to a top-down, "birds eye" view
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")

    # calculate the perspective transform matrix and warp
    # the perspective to grab the screen
    M = cv2.getPerspectiveTransform(rect, dst)
    warp = cv2.warpPerspective(orig, M, (maxWidth, maxHeight))
    return warp

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

def pidController(currentValue, desiredValue, Kp, Kd, Ki):
    global e_prev
    global e_int
    error = currentValue - desiredValue
    e_int = e_int + error
    e_diff = error - e_prev
    pid = Kp * error + Ki * e_int + Kd * e_diff
    e_prev = error
    return pid

