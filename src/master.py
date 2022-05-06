#!/usr/bin/env python3
# +----------------------------------------------------------------------------
# |
# | United States Air Force Academy     __  _______ ___    _________
# | Dept of Electrical &               / / / / ___//   |  / ____/   |
# | Computer Engineering              / / / /\__ \/ /| | / /_  / /| |
# | 2354 Fairchild Drive Ste 2F6     / /_/ /___/ / ___ |/ __/ / ___ |
# | USAF Academy, CO 80840           \____//____/_/  |_/_/   /_/  |_|
# |
# | ---------------------------------------------------------------------------
# |
# | FILENAME      : master.py
# | AUTHOR(S)     : C1C Anthony Tolbert
# | CONTACT       : (559) 326-4289, ajtolbert63@yahoo.com
# | CREATED       : 11 Jan 2022
# | Last Update   : 08 Apr 2022
"""
This module defines a class for simple robot navigation and functions to support operating multiple instances of the
 class.

The master class abstracts away much of the ROS functionality needed for basic robot navigation. The class was created
primarily with the waterfall implementation in mind, but should be able to be used effectively with other types of
navigation and collision avoidance.

When using navigating robots, a list of the Master class should be created to effectively leverage the additional
functions in this module.

This script requires:
    * rospy
    * time
    * geometry_msgs
    * usafalog

This file contains 5 standalone functions and 1 class:
    Functions
    ---------
    * stop_pub : Stop all bots and stop publishing to the destination topic
    * start_bots : Release all bots at the same time and allow them to seek their destination positions
    * assign_bots : Assign the destination points to all robots from parallel lists.
    * all_bots_found : Reports when all bots are no longer in the BOOT state, meaning either their positions have been
       reported or they've timed out
    * start_positions : Gives a set of parallel list containing the x and y positions of the robots when the function is
       called

    Class
    -----
    * Master : A class for implementation of navigation for usafabots using ROS.
"""
import time

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

import usafalog

logger = usafalog.CreateLogger(__name__)
# Global Variables
CLOSE_DIST = .25  # meters
DONE_DIST = .10  # meters
TIMEOUT_THRESH = 10  # seconds

# Manual enumeration of robot states
UNAVAIL = -1  # The robot was not able to be found in time
BOOT = 0  # The robot has turned on but not found by RR and not yet timed-out
WAITING = 1  # It is not the robot's time yet
WORKING = 2  # The robot is trying to get to its destination
CLOSE = 3  # The robot is close enough to release the next bot
DONE = 4  # The robot is at its destination

BREAD_PERIOD = 2  # (s) how often to drop a breadcrumb when robot is working
PUBLISH_PERIOD = .1  # (s) how often to publish a point to the dest_pos topic

# DFEC from inside to outside
x_dest = [2.3, 2.0, 1.6, 1.30, 1.30, 1.0, 1.0, 1.0, 2.0, 2.3, 2.6, 3.0, 3.3, 3.6, 4.0, 4.0, 4.0, 4.6, 4.6, 3.6, 3.3,
          3.3, 3.0, 3.0, 2.0]
y_dest = [2.5, 2.5, 2.5, 2.75, 2.25, 2.0, 2.5, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 2.0, 2.5, 2.0, 3.0, 2.0, 2.5,
          2.0, 2.5, 2.0, 2.0]


class Master:
    """
        A class for implementation of navigation for usafabots using ROS.

        Attributes
        ----------
        curr_pos : geometry_msgs.msg.Pose()
            A pose msg containing data on the current position of the robot.
             To access {x,y,z} : self.curr_pos.position.{x,y,z}
        dest_pos : geometry_msgs.msg.Point()
            Assigned final destination position. Not necessarily what is being published on the topic.
             (see callbackPublisher)
        name : str
            Assigned robot name for logging and other references (i.e. "usafabot22")
        pos_err : float
            Measurement, in meters, of the radial distance between the robot and its assigned destination
        x_avg : list
            Collection of x points used by the error_checking module to determine position error
        y_avg :
            Collection of y points used by the error_checking module to determine position error
        timeout : bool
            Flag for determining if a robot has taken too long to be discovered by the position sensors
        time : float
            Time from when the robot starts to when it reaches its destination position -- used for data collection
        dist : float
            Radial distance from the robot to the destination position
        state : int
            Manually enumerate states defining what the robot is doing
        dest_set : bool
            Flag determining if a robot has been assigned a destination position or should be skipped by the controller
        lock : bool
            Flag determining if the robot's destination position should be published or if (0,0,0) should be published,
             which freezes the robot (see callbackPublisher)
        breadcrumbs : list of [x,y] tuples
            Periodic recordings of robots position while moving. Used to plot robot's real path.
        BC_counter : int
            Keeps track of how many points have been recorded after the robot is 'CLOSE' to its destination
        publish : bool
            Flag to determine if the publisher should continue publishing any points (could reduce ROS errors on close)

        Methods
        -------
        * setGroundDestPosition(x, y) : Assigns a 2-D coordinate to the geometry_msgs.msg.Point() data type assigned to
           the robot
        * callback_currPos(data) : Periodically called function to update the robot's current position with information
            from the RoadRunner
        * stop() : Stop the robot from moving, regardless of if the is destination set
        * start() : Start publishing the destination position so the onboard controller drives the robot
        * callbackPublisher(event) : Automatically and Periodically publish a destination position for the robot
        * drop_breadcrumbs(event) : Automatically and Periodically record the robot's current position (drop a
           breadcrumb) and append the data to the robot's x and y breadcrumb list
        """

    def __init__(self, name):
        """
        Parameters
        ----------
        name : str
            Assigned robot name for logging and other references (i.e. usafabot22)
        """
        self.curr_pos = Pose()
        self.dest_pos = Point()
        self.name = name
        self.pos_err = 0
        self.x_avg = []
        self.y_avg = []
        self.timeout = False
        self.time = 0
        self.dist = float('Inf')
        self.state = BOOT
        self.dest_set = False
        self.lock = True
        self.breadcrumbs = [[], []]
        self.BC_counter = 0
        self.publish = True

        # -----------------------------------------------------------------------------
        # Topics and Timers
        # -----------------------------------------------------------------------------
        # Method to publish destination to the robot
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size=10)
        # Automatically & continuously publish destination position
        rospy.Timer(rospy.Duration(PUBLISH_PERIOD), self.callbackPublisher)
        # Periodically record current position (breadcrumb trail)
        rospy.Timer(rospy.Duration(BREAD_PERIOD), self.drop_breadcrumbs)
        # Listen for the bots' current position to the controller
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)
        # -------------------------------------------------------------------------------

    # Class Functions
    # -------------------------------------------------------------------------------
    def setGroundDestPosition(self, x, y):
        """
        Assigns a 2-D coordinate to the geometry_msgs.msg.Point() data type assigned to the robot

        Parameters
        ----------
        x : float
            x-position in the range of [0, 5.5]
        y : float
            y-position in the range of [0, 6]
        """
        self.dest_pos.x = x
        self.dest_pos.y = y
        self.dest_set = True
        logger.debug(f"{self.name} dest_pos set to {self.dest_pos}")

    def callback_currPos(self, data):
        """
        Periodically called function to update the robot's current position from the RoadRunner

        This function also includes functionality to update the robot's state based on:
            i.      If the robot has been "found" by the positioning system
            ii.     If the robot is approaching its destination
            iii.    If the robot is at its destination
            iv.     If the robot has timed out and is considered unfound

        Parameters
        ----------
        data : Pose()
            Automatically generated geometry_msgs.msg.Pose() data passed by the ROS timer
        """
        self.curr_pos.position.x = round(data.position.x, 3)  # must assign each component separately in order to round
        self.curr_pos.position.y = round(data.position.y, 3)
        self.curr_pos.orientation.z = round(data.orientation.z, 3)
        # Automatically update the distance to destination
        self.dist = ((self.dest_pos.x - self.curr_pos.position.x) ** 2 + (
                self.dest_pos.y - self.curr_pos.position.y) ** 2) ** 0.5

        if self.dist < CLOSE_DIST: self.state = CLOSE
        if self.dist < DONE_DIST: self.state = DONE
        if self.state == DONE and not self.lock:
            self.lock = True
            self.time = time.perf_counter() - self.time
            logger.info(f"{self.name} complete in {round(self.time, 4)} (s)")

        # initialize timer to keep track of how long it takes to "find" the robot
        tic = time.perf_counter()
        while self.state == BOOT:
            if time.perf_counter() - tic > TIMEOUT_THRESH:  # Robot has timed-out
                self.timeout = True
                self.state = UNAVAIL
                logger.warning(f"Timeout: {self.name}")
                break
            elif data.position.x != 0 or data.position.y != 0:  # Robot has been found
                toc = time.perf_counter()
                t = toc - tic
                logger.debug(
                    f"Found {self.name} in {round(t, 4)} (s) at ({round(self.curr_pos.position.x, 2)}, {round(self.curr_pos.position.y, 2)})")
                self.state = WAITING

    def stop(self):
        """Stop the robot from moving, regardless of if the is destination set"""
        self.lock = True
        logger.debug(f"{self.name} stopped")

    def start(self):
        """Start publishing the destination position so the onboard controller drives the robot"""
        self.time = time.perf_counter()
        self.lock = False
        self.state = WORKING
        logger.info(f"{self.name} started")

    def callbackPublisher(self, event):
        """
        Automatically and Periodically publish a destination position for the robot

        Locks robot if "lock" attribute is set by taking advantage of the fact the robot will not move when all 0s are
         published as its destination position. This functionality can be seen in the usafabot's controller code
         (robotics_ws/src/usafabot/src/controller.py)

        Parameters
        ----------
        event : rostimer.event (ignored)
            Automatically generated passed by the ROS timer
        """
        if self.publish:
            if self.lock:
                self.pub.publish(0, 0, 0)
            else:
                self.pub.publish(self.dest_pos)

    def drop_breadcrumbs(self, event):
        """
        Automatically and Periodically record the robot's current position (drop a breadcrumb) and append the data to
         the robot's x and y breadcrumb list

        Parameters
        ----------
        event : rostimer.event (ignored)
            Automatically generated data passed by the ROS timer
        """
        if self.state == WORKING or self.state == CLOSE:
            self.breadcrumbs.append([self.curr_pos.position.x, self.curr_pos.position.y])
        elif self.state == DONE and self.BC_counter < 2:  # drop 2 crumbs after getting to point
            self.breadcrumbs.append([self.curr_pos.position.x, self.curr_pos.position.y])
            self.BC_counter += 1
    # -------------------------------------------------------------------------------


def stop_pub(bots: list):
    """
    Stop all bots and stop publishing to the destination topic

    Parameters
    ----------
    bots : list
        list of robots (Master Class) to be stopped
    """
    for bot in bots:
        assert isinstance(bot, Master)
        bot.stop()
        bot.publish = False
    logger.info('locked all bots')


def start_bots(bots: list):
    """
    Release all bots at the same time and allow them to seek their destination positions

    Parameters
    ----------
    bots : list
        list of robots (Master Class) to be started
    """
    for bot in bots:
        assert isinstance(bot, Master)
        bot.start()
    logger.info("UNlocked all bots")


def assign_bots(bots: list, xdest=None, ydest=None):
    """
    Assign the destination points to all robots from parallel lists.

    Parameters
    ----------
    bots : list
        list of robots (Master Class) to be assigned (should be all robots)
    xdest : list, optional
        list of x positions in order of the robots (x0, x1, x2, ... xn)
        When not passed, the x positions for DFEC are used
    ydest : list, optional
        list of y positions in order of the robots (y0, y1, y2, ... yn)
        When not passed, the y positions for DFEC are used
    """
    if xdest is None:
        xdest = x_dest
    if ydest is None:
        ydest = y_dest
    for bot, xdestintation, ydestination in zip(bots, xdest, ydest):
        assert isinstance(bot, Master)
        bot.setGroundDestPosition(xdestintation, ydestination)


def all_bots_found(bots: list):
    """
    Reports when all bots are no longer in the BOOT state, meaning either their positions have been reported or
    they've timed out

    Parameters
    ----------
    bots : list
        list of robots (Master Class)

    Returns
    --------
    boolean of if bots are found or not

    """
    not_found = 0
    for bot in bots:
        if bot.state == BOOT:
            not_found += 1
    if not_found > 0:
        return False
    logger.info("All bots found")
    return True


def start_positions(bots: list):
    """
    Gives a set of parallel lists containing the x and y positions of the robots when the function is called

    Parameters
    ----------
    bots : list
        list of robots (Master Class) to get position
    Returns
    ----------
    x : list
        list of x positions in order of the robots (x0, x1, x2, ... xn)
    y : list
        list of y positions in order of the robots (y0, y1, y2, ... yn)
    """
    x = []
    y = []
    for bot in bots:
        if not bot.timeout:  # Only include found bots
            x.append(bot.curr_pos.position.x)
            y.append(bot.curr_pos.position.y)
    logger.debug(f"found {len(x)} start positions")
    return x, y
