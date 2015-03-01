# Joshua Emele <jemele@acm.org>
# Tristan Monroe <twmonroe@eng.ucsd.edu>
import cv2
import numpy as np
import time

"""OpenCV v4l2 color tracker."""
class CVTracker:

    """Information about the last tracked in-frame object."""
    timestamp, x, y, area = None, None, None, None

    """Initialize capture device and calibration controls."""
    def __init__(self):

        # Create and initialize capture device
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,240)
        self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,320)

        # Create controls
        cv2.namedWindow('c')
        def nothing(x):
            pass
        cv2.createTrackbar('b0','c',15,255,nothing)
        cv2.createTrackbar('h0','c',60,179,nothing)
        cv2.createTrackbar('s0','c',20,255,nothing)
        cv2.createTrackbar('v0','c',20,255,nothing)
        cv2.createTrackbar('h1','c',160,179,nothing)
        cv2.createTrackbar('s1','c',200,255,nothing)
        cv2.createTrackbar('v1','c',200,255,nothing)

    """Capture and process an image frame.""" 
    def process_frame(self):
        _, frame = self.cap.read()

        #converting to HSV
        b0 = cv2.getTrackbarPos('b0','c')
        frame = cv2.medianBlur(frame,b0)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        # get masking information from the user
        lower = np.array([
            cv2.getTrackbarPos('h0','c'),
            cv2.getTrackbarPos('s0','c'),
            cv2.getTrackbarPos('v0','c')])
        upper = np.array([
            cv2.getTrackbarPos('h1','c'),
            cv2.getTrackbarPos('s1','c'),
            cv2.getTrackbarPos('v1','c')])
        mask = cv2.inRange(hsv, lower, upper)

        # https://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        # apply open and closing morpholophy
        # open to remove noise
        # closing to fill holes
        kernel = np.ones((b0,b0),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # compute moments meeting area requirements
        # http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
        # http://opencvpython.blogspot.com/2012/06/contours-2-brotherhood.html
        moments = cv2.moments(mask)
        area = int(moments['m00'])
        if area > 100000:
            self.area = area
            self.x, self.y = int(moments['m10']/area), int(moments['m01']/area)
            self.timestamp = time.time()

            # fraw the centroid on the frame for reference
            cv2.circle(frame,(self.x,self.y),10,(255,255,255),-1)

        # show the frame and the mask
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)

# Application entry point.
if __name__ == '__main__':

    c = CVTracker()
    while True:

         # capture and process a frame
        c.process_frame()

        # bail on escape
        k = cv2.waitKey(10) & 0xFF
        if k == 27:
            break


