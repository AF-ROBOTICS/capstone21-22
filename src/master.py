import time

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

import usafalog

logger = usafalog.CreateLogger(__name__)
# Global Variables
DEST_DIST = .25  # meters
DONE_DIST = .10  # meters
TIMEOUT_THRESH = 10  # seconds
# Manual enumeration of states
UNAVAIL = -1  # The robot was not able to be found in time
BOOT = 0  # The robot has turned on but not found by RR or timed-out
WAITING = 1  # It is not the robot's time yet
WORKING = 2  # The robot is trying to get to its dest
CLOSE = 3  # The robot is close enough to release the next bot
DONE = 4  # The robot is at its destination

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
        self.dist = 999999999.015
        self.state = BOOT
        self.dest_set = False
        self.lock = True
        # -----------------------------------------------------------------------------
        # Topics and Timers
        # -----------------------------------------------------------------------------
        # Publish to the controller
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size=10)
        rospy.Timer(rospy.Duration(.1), self.callbackPublisher)  # Automatically publish dest pos

        # Listen for the bots' current position to the controller
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)

        # -------------------------------------------------------------------------------

    # Class Functions
    # -------------------------------------------------------------------------------
    def setGroundDestPosition(self, x, y):
        # Data based on drone position
        self.dest_pos.x = x
        self.dest_pos.y = y
        self.dest_set = True
        logger.debug(f"{self.name} dest_pos set to {self.dest_pos}")

    def callback_currPos(self, data):
        self.curr_pos.position.x = round(data.position.x, 3)
        self.curr_pos.position.y = round(data.position.y, 3)
        self.curr_pos.orientation.z = round(data.orientation.z, 3)
        self.dist = ((self.dest_pos.x - self.curr_pos.position.x) ** 2 + (
                self.dest_pos.y - self.curr_pos.position.y) ** 2) ** 0.5
        if self.dist < DEST_DIST: self.state = CLOSE
        if self.dist < DONE_DIST: self.state = DONE
        if self.state == DONE and not self.lock:
            self.lock = True
            self.time = time.perf_counter() - self.time
            logger.info(f"{self.name} complete in {round(self.time, 4)} (s)")
        # logger.info(f"{self.dist}")
        tic = time.perf_counter()
        while self.state == BOOT:
            if time.perf_counter() - tic > TIMEOUT_THRESH:
                self.timeout = True
                self.state = UNAVAIL
                logger.warning(f"Timeout: {self.name}")
                break
            elif data.position.x != 0 or data.position.y != 0:
                toc = time.perf_counter()
                t = toc - tic
                logger.debug(
                    f"Found {self.name} in {round(t, 4)} (s) at ({round(self.curr_pos.position.x, 2)}, {round(self.curr_pos.position.y, 2)}")
                self.state = WAITING

    def stop(self):
        self.lock = True
        logger.debug(f"{self.name} stopped")

    def start(self):
        self.time = time.perf_counter()
        self.lock = False
        logger.info(f"{self.name} started")

    def callbackPublisher(self, event):
        # logger.debug(f"Publishing {self.name}")
        if self.lock:
            self.pub.publish(0, 0, 0)
        else:
            self.pub.publish(self.dest_pos)


def stop_bots(bots: list):
    for bot in bots:
        assert isinstance(bot, Master)
        bot.stop()
    logger.info('locked all bots')


def start_bots(bots: list):
    for bot in bots:
        assert isinstance(bot, Master)
        bot.start()
    logger.info("UNlocked all bots")


def assign_bots(bots: list, xdest=None, ydest=None):
    if xdest is None:
        xdest = x_dest
    if ydest is None:
        ydest = y_dest
    for bot, xdestintations, ydestinations in zip(bots, xdest, ydest):
        assert isinstance(bot, Master)
        bot.setGroundDestPosition(xdestintations, ydestinations)


def all_bots_found(bots: list):
    not_found = 0
    for bot in bots:
        if bot.state == BOOT:
            not_found += 1
    if not_found > 0:
        return False
    logger.info("All bots found")
    return True


def start_positions(bots: list):
    x = []
    y = []
    for bot in bots:
        if not bot.timeout:  # Only include found bots
            x.append(bot.curr_pos.position.x)
            y.append(bot.curr_pos.position.y)
    logger.debug(f"found {len(x)} start positions")
    return x, y
