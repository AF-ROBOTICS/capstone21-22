import time

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from usafalog import *

# Global Variables
DEST_DIST = .25  # meters
DONE_DIST = .10  # meters
TIMEOUT_THRESH = 10  # seconds
# Kill state magic number
KILL_SIG = 22

# DFEC from inside to outside
x_dest = [2.3, 2, 1.6, 1.3, 1.3, 1, 1, 1, 2, 2.3, 2.6, 3, 3.3, 3.6, 4, 4, 4, 4.6, 4.6, 3.6, 3.3, 3.3, 3, 3, 2]
y_dest = [2.5, 2.5, 2.5, 2.75, 2.25, 2, 2.5, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2.5, 2, 3, 2, 2.5, 2, 2.5, 2, 2]


class Master:

    def __init__(self, name):
        self.curr_pos = Pose()
        self.dest_pos = Point()
        self.name = name
        self.pos_err = 0
        self.x_avg = []
        self.y_avg = []
        self.timeout = False
        self.time = 0
        self.dist = ((self.dest_pos.x - self.curr_pos.position.x) ** 2 + (self.dest_pos.y - self.curr_pos.position.y) ** 2) ** 0.5
        self.done = self.dist < DONE_DIST
        self.close = self.dist < DEST_DIST
        # -----------------------------------------------------------------------------
        # Topics and Timers
        # -----------------------------------------------------------------------------
        # Publish to the controller
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size=10)
        rospy.Timer(rospy.Duration(.1), self.pub.publish(self.dest_pos))  # Automatically publish dest pos

        # Listen for the bots' current position to the controller
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)

        # -------------------------------------------------------------------------------

    # Class Functions
    # -------------------------------------------------------------------------------
    def setGroundDestPosition(self, x, y):
        # Data based on drone position
        self.dest_pos.x = x
        self.dest_pos.y = y
        logger.debug(self.name, "dest_pos set to", self.dest_pos)

    def callback_currPos(self, data):
        self.curr_pos.position.x = round(data.position.x, 3)
        self.curr_pos.position.y = round(data.position.y, 3)
        self.curr_pos.orientation.z = round(data.orientation.z, 3)
        logger.debug("Waiting bot: " + self.name)
        tic = time.perf_counter()
        while data.position.x == 0 and data.position.y == 0:
            if time.perf_counter() - ticc > TIMEOUT_THRESH:
                self.timeout = True
                logger.warning("Timeout: " + self.name)
                break
            elif data.position.x or data.position.y:
                toc = time.perf_counter()
                t = toc - tic
                logger.info("Found", self.name, 'in', round(t, 4), '(s)')

    def stop(self):
        self.setGroundDestPosition(KILL_SIG, KILL_SIG)
        self.time = time.perf_counter() - self.time
        logger.info(self.name, "complete in", round(self.time, 4), "(s)")
        logger.debug(self.name, "stopped")

    def start(self):
        self.setGroundDestPosition(-KILL_SIG, -KILL_SIG)
        self.time = time.perf_counter()
        logger.debug("starting timer for", self.name)
        logger.debug(self.name, "stopped")


def stop_bots(bots: list):
    for bot in bots:
        assert isinstance(bot, Master)
        bot.stop()
    logger.debug('locked all bots')


def start_bots(bots: list):
    for bot in bots:
        assert isinstance(bot, Master)
        bot.start()
    logger.debug("UNlocked all bots")


def assign_bots(bots: list, xdest=x_dest, ydest=y_dest):
    for bot, xdest, ydest in zip(bots, x_dest, y_dest):
        assert isinstance(bot, Master)
        logger.debug("Dest set for:" + bot.name)
        bot.setGroundDestPosition(xdest, ydest)
