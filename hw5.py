# Joshua Emele <jemele@acm.org>
# Tristan Monroe <twmonroe@eng.ucsd.edu>
import array
import logging
import serial
import struct
import threading
import time

""" Interact with an irobot over serial."""
class IRobot:
    """A serial device used to communicate with the irobot.""" 
    device = serial.Serial('/dev/ttyO1',57600)

    """If valid, a threading.Timer object for sensor polling."""
    timer = None

    def __init__(self):
        logging.debug('starting irobot')
        self.mode_start()

    def mode_start(self):
        logging.debug('mode start')
        c=array.array('B',[128])
        self.device.write(c.tostring())

    def mode_safe(self):
        logging.debug('mode safe')
        c=array.array('B',[131])
        self.device.write(c.tostring())

    def mode_full(self):
        logging.debug('mode full')
        c=array.array('B',[132])
        self.device.write(c.tostring())

    def sensor_start(self):
        self.sensor_stop()
        self.timer = threading.Timer(1.0, self._sensor_poll)
        logging.debug("sensor start")
        self.timer.start()

    def sensor_stop(self):
        if self.timer is not None:
            logging.debug("sensor stop")
            self.timer.cancel()
            self.timer = None

    def drive(self, left_mm_per_s, right_mm_per_s):
        logging.debug("drive %d,%d" % (left_mm_per_s, right_mm_per_s))
        pass

    def stop(self):
        logging.debug("stop")
        self.drive(0,0)

    def _sensor_poll(self):
        logging.debug("sensor poll")

        logging.debug("flushing serial")
        self.device.flushInput()
        self.device.flush()

        # Bumps and Wheel Drops Packet ID: 7 Data Bytes: 1
        # Wall Packet ID: 8 Data Bytes: 1# 
        # Distance Packet ID: 19 Data Bytes: 2
        logging.debug("query list")
        c=[149,3,7,8,19]
        self.device.write(array.array('B',c).tostring())

        f='<BBH'
        logging.debug("reading sensor: %d bytes" % (struct.calcsize(f)))
        d=self.device.read(struct.calcsize(f))
        b=struct.unpack(f,d)
        logging.debug(b)
        self.sensor_bumper = b[0]
        self.sensor_wall = b[1]
        self.sensor_distance = b[2]
        self.sensor_timestamp = time.time()

        # Stop movement on bump
        if self.sensor_bumper:
            logging.debug("bump detected, stopping")
            self.stop()

        logging.debug("rescheduling poll")
        self.sensor_start()

# Application entry point.
if __name__ == '__main__':

    # set logging to kill
    logging.getLogger().setLevel(logging.DEBUG)

    # initialize the irobot
    r=IRobot()
    r.mode_safe()
    r.mode_full()

    # start the timer and sleep for a while
    r.sensor_start()
    time.sleep(5)
    r.sensor_stop()

