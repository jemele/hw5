import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,320)

def nothing(x):
    pass

# Creating a window for later use
cv2.namedWindow('result')

# Creating track bar
# Tuned to my blue rei shirt
cv2.createTrackbar('b0','result',15,255,nothing)
cv2.createTrackbar('h0','result',60,179,nothing)
cv2.createTrackbar('s0','result',20,255,nothing)
cv2.createTrackbar('v0','result',20,255,nothing)
cv2.createTrackbar('h1','result',160,179,nothing)
cv2.createTrackbar('s1','result',200,255,nothing)
cv2.createTrackbar('v1','result',200,255,nothing)

while(1):

    _, frame = cap.read()

    #converting to HSV
    b0 = cv2.getTrackbarPos('b0','result')
    frame = cv2.medianBlur(frame,b0)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    h0 = cv2.getTrackbarPos('h0','result')
    s0 = cv2.getTrackbarPos('s0','result')
    v0 = cv2.getTrackbarPos('v0','result')

    h1 = cv2.getTrackbarPos('h1','result')
    s1 = cv2.getTrackbarPos('s1','result')
    v1 = cv2.getTrackbarPos('v1','result')

    # Normal masking algorithm
    lower = np.array([h0,s0,v0])
    upper = np.array([h1,s1,v1])
    mask = cv2.inRange(hsv, lower, upper)

    # https://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
    # apply open and closing morpholophy
    # open to remove noise
    # closing to fill holes
    kernel = np.ones((b0,b0),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)

    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()
