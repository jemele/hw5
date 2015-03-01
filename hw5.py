# Joshua Emele <jemele@acm.org>
# Tristan Monroe <twmonroe@eng.ucsd.edu>
import array
import cv2
import logging
import numpy as np
import serial
import struct
import sys
import threading
import time

""" Interact with an irobot over serial."""
class IRobot:

    """A serial device used to communicate with the irobot.""" 
    device = serial.Serial('/dev/ttyO1',57600)

    """A lock used to serialize access to the device."""
    lock = threading.Lock()

    """If valid, a threading.Timer object for sensor polling."""
    timer = None

    """Integrated distance and angle derived from sensor poll."""
    distance, angle = 0, 0

    """Initialize the device."""
    def __init__(self):
        logging.debug('starting irobot')
        self.mode_start()

    """Start the irobot."""
    def mode_start(self):
        logging.debug('mode start')

        try:
            self.lock.acquire()
            self.device.flushInput()
            self.device.flush()
            c=array.array('B',[128])
            self.device.write(c.tostring())

        finally:
            self.lock.release()

    """Enter safe mode."""
    def mode_safe(self):
        logging.debug('mode safe')

        try:
            self.lock.acquire()
            self.device.flushInput()
            self.device.flush()
            c=array.array('B',[131])
            self.device.write(c.tostring())

        finally:
            self.lock.release()

    """Enter full mode."""
    def mode_full(self):
        logging.debug('mode full')

        try:
            self.lock.acquire()
            self.device.flushInput()
            self.device.flush()
            c=array.array('B',[132])
            self.device.write(c.tostring())

        finally:
            self.lock.release()

    """Start the sensor polling timer."""
    def sensor_start(self, period_s):
        self.sensor_period_s = period_s
        self.sensor_stop()
        self.timer = threading.Timer(period_s, self._sensor_poll)
        logging.debug("sensor start")
        self.timer.start()

    """Stop the sensor polling timer."""
    def sensor_stop(self):
        if self.timer is not None:
            logging.debug("sensor stop")
            self.timer.cancel()
            self.timer = None

    """Drive each wheel at the specified rate."""
    def drive(self, left_mm_per_s, right_mm_per_s):
        logging.info("drive %d,%d" % (left_mm_per_s, right_mm_per_s))

        # XXX This shouldn't be necessary, but byte ordering appears to be a
        # problem.
        f='<BBBBB'
        l=[(left_mm_per_s>>8)&0xff,(left_mm_per_s&0xff)]
        r=[(right_mm_per_s>>8)&0xff,(right_mm_per_s&0xff)]
        c=struct.pack(f,145,l[0],l[1],r[0],r[1])

        try:
            self.lock.acquire()
            self.device.flushInput()
            self.device.flush()
            self.device.write(c)
        finally:
            self.lock.release()

    """Stop driving."""
    def stop(self):
        logging.debug("stop")
        self.drive(0,0)

    """Poll the irobot for sensor data."""
    def _sensor_poll(self):
        logging.debug("sensor poll")

        # Bumps and Wheel Drops Packet ID: 7 Data Bytes: 1
        # Wall Packet ID: 8 Data Bytes: 1# 
        # Distance Packet ID: 19 Data Bytes: 2
        # Angle Packet ID: 20 Data Bytes: 2
        # the sensor data, once read
        c=[149,4,7,8,19,20]
        f='<BBBBBB'
        data = None

        try:
            self.lock.acquire()
            self.device.flushInput()
            self.device.flush()
            self.device.write(array.array('B',c).tostring())
            data = self.device.read(struct.calcsize(f))

        finally:
            self.lock.release()

        b=struct.unpack(f,data)
        self.sensor_bumper = b[0]
        self.sensor_wall = b[1]
        self.sensor_distance = int((b[2]<<8)|b[3])
        self.sensor_angle = int((b[4]<<8)|b[5])
        self.sensor_timestamp = time.time()

        # integrate distance and angle
        self.distance += self.sensor_distance
        self.angle += self.sensor_angle
        logging.info("%d: bumper %d wall %d distance %d,%d angle %d,%d" % \
                (self.sensor_timestamp,self.sensor_bumper,self.sensor_wall,
                 self.sensor_distance,self.distance,
                 self.sensor_angle,self.angle))

        # Stop movement on bump
        if self.sensor_bumper:
            logging.debug("bump detected, stopping")
            self.stop()

        # reschedule the next sensor poll
        self.sensor_start(self.sensor_period_s)


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

    # set logging to kill
    logging.getLogger().setLevel(logging.INFO)

    # initialize the irobot
    # stay in safe mode, since there's no need for full
    r = IRobot()
    r.mode_safe()
    r.mode_full()

    # poll the irobot sensor
    r.sensor_start(0.4)

    # start a bump test
    r.drive(10,10)

    # initialize the tracker
    c = CVTracker()
    while True:

        # capture and process a frame
        c.process_frame()

        # bail on escape
        k = cv2.waitKey(100) & 0xFF
        if k == 27:
            break

    # stop the irobot
    r.stop()
    r.sensor_stop()
        
