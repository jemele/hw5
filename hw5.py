# Joshua Emele <jemele@acm.org>
# Tristan Monroe <twmonroe@eng.ucsd.edu>
import array
import cv2
import logging
import math
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
        c=struct.pack('<B',145)
        c+=struct.pack('>hh',right_mm_per_s,left_mm_per_s)

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

        try:
            self.lock.acquire()
            self.device.flushInput()
            self.device.flush()

            # Bumps and Wheel Drops Packet ID: 7 Data Bytes: 1
            # Wall Packet ID: 8 Data Bytes: 1
            # Distance Packet ID: 19 Data Bytes: 2
            # Angle Packet ID: 20 Data Bytes: 2
            # the sensor data, once read
            c=[149,4,7,8,19,20]
            self.device.write(array.array('B',c).tostring())
            self.sensor_timestamp = time.time()

            # read off the single byte fields first
            f='<BB'
            d=self.device.read(struct.calcsize(f))
            b=struct.unpack(f,d)
            self.sensor_bumper = b[0]
            self.sensor_wall = b[1]

            # read off the multibyte fields next
            f='>hh'
            d=self.device.read(struct.calcsize(f))
            b=struct.unpack(f,d)
            self.sensor_distance = b[0]
            self.sensor_angle = b[1]

        finally:
            self.lock.release()

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

    """Capture and discard an image frame."""
    def flush_frame(self):
        self.cap.read()

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

# Acquire an object.
# On acquisition, stop and return.
# Acquisition can be aborted, but will otherwise run forever.
def acquire(robot, tracker, rate_mm_s = 10):

    # start the acquisition loop
    # begin rotation until the object is detected
    # if a full rotation occurs without seeing the object, panic.
    # begin *slow* rotation
    detects, detect_threshold = 0, 3
    robot.drive(rate_mm_s, -rate_mm_s)
    while True:

        # capture and process a frame
        now = time.time()
        tracker.process_frame()

        # check to see if an object was detected
        # if so, bail
        timestamp = tracker.timestamp
        if timestamp is not None and timestamp > now:
            detects += 1
            logging.info("detect %d" % (detects))
            if detects >= detect_threshold:
                logging.info("acquired object")
                robot.stop()
                return
        else:
            logging.info("searching...")

        # check for abort
        k = cv2.waitKey(50) & 0xff
        if k == 27:
            logging.info("acquisition abort")
            sys.exit(0)

# Calculate the range and bearing of the object being tracked.
# Return the tuple (range_mm, bearing_degrees)
def track(tracker, timeout_s = 1.0):

    # attempt to track the target
    # read the tracker and make sure we have a target
    start = time.time()
    while tracker.timestamp < start:
        tracker.process_frame()
        
        # check for timeout
        if (time.time() - start) > timeout_s:
            logging.info("track timeout")
            return None

        # check for abort
        k = cv2.waitKey(50) & 0xff
        if k == 27:
            logging.info("tracking abort")
            sys.exit(0)

    # we found something!
    logging.info("tracker x %d y %d", tracker.x, tracker.y)

    # calculate range
    y = tracker.y
    track_range_mm = 762.0/math.tan(2.0*math.pi*((80.0-(y/5.27))/360.0))

    # calculate bearing (radians)
    x = tracker.x - 160
    track_bearing_degrees = (x/7.4)
    return (track_range_mm, track_bearing_degrees)

"""Return +1 if >= 0, -1 otherwise."""
def sign(x):
    if x < 0:
        return -1
    return 1

# Application entry point.
if __name__ == '__main__':

    # set logging to kill
    logging.getLogger().setLevel(logging.INFO)

    # disable the laser
    open('/sys/class/gpio/gpio20/value','w').write('1')

    # initialize the irobot
    r = IRobot()
    r.mode_safe()
    r.mode_full()

    # initialize the tracker and calibrate
    c = CVTracker()
    while True:
        c.process_frame()
        k = cv2.waitKey(50) & 0xff
        if chr(k) == 'c': # continue
            logging.debug("calibration complete")
            break
        elif k == 27: # escape
            logging.info("calibration abort")
            sys.exit(0)

    try:
        # poll the irobot sensor
        r.sensor_start(0.4)

        # attempt to track the object
        # if this fails, go into acquisition
        # on acqusition, we'll continue tracking...
        t_prev, t = None, None
        while True:

            # track our object, if the object cannot be tracked, acquire.
            t = track(c)
            if t is None:
                # if the last known track is available, use the last known
                # bearing to decide which direction we acquire
                rate = 30
                if t_prev is not None:
                    rate *= sign(t_prev[1])

                logging.info("track lost... acquiring... %d" % (rate))
                acquire(r,c,rate)
                
                # discard a frame to make sure we're not using stale data
                c.flush_frame()
                continue

            # we have track ... seek and destroy!
            t_prev = t
            logging.info("tracking range %f bearing %f" % (t))

            # scale our forward movement based on the range
            # XXX what's an ideal range and bearing error?
            if t[0] < 3000 and math.fabs(t[1]) < 3:
                logging.info("firing laser")
                open('/sys/class/gpio/gpio20/value','w').write('1')
                sys.exit(0)

            # rotate toward the object while moving forward
            rate = 20
            right_delta = t[1]/2
            r.drive(rate - right_delta, rate + right_delta)

    finally:
        # stop the irobot
        r.stop()
        r.sensor_stop()
