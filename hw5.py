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

    def _sensor_poll(self):
        logging.debug("sensor poll")

        logging.debug("flushing serial")
        self.device.flushInput()
        self.device.flush()

        logging.debug("query list")
        # Bumps and Wheel Drops Packet ID: 7 Data Bytes: 1
        # Wall Packet ID: 8 Data Bytes: 1# 
        # Distance Packet ID: 19 Data Bytes: 2
        c=[149,3,7,8,19]
        self.device.write(array.array('B',c).tostring())
        time.sleep(0.1)

        f='<BBH'
        logging.debug("reading sensor: %d bytes" % (struct.calcsize(f)))
        d=self.device.read(struct.calcsize(f))
        logging.debug("read: %d bytes" % (len(d)))
        b=struct.unpack(f,d)
        print b

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
    time.sleep(10)
    r.sensor_stop()

