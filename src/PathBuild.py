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
# | FILENAME      : PathBuild.py
# | AUTHOR(S)     : C1C Anthony Tolbert
# | CREATED       : 5 Feb 2022
# | Last Update   : 12 Apr 2022
"""
This module defines classes and functions useful for determining destination, origin pairs for waterfall movement.

This module primarily uses a recursive backtracking algorithm to find paths for robots that do not intersect with robots
 already at their final destination positions or even come within a user defined buffer distance to allow for variation
 in the position sensor's measurement and non-ideal driving.

The functions in this program, namely solve(), can be very computationally intensive and can take a lot of time to
 complete. For this reason, a .json cache is used to store successful runs and to serve as a starting point for
 subsequent runs of the same input.


This script requires:
    * json
    * os
    * random (debugging only)
    * time
    * copy
    * usafalog

This file contains 9 standalone functions and 2 class:
    Functions
    ---------
    * lines_intersect : Determines if and where two line segments intersect
    * line_close_to_point : Determines if a line segment is ever closer than the BUFFER_DIST to a point
    * animate : Creates a pyplot to show proposed line and all established endpoints
    * is_valid_line : Compares line to all established endpoints and determines if the path is valid to be taken by the
       usafabot
    * solve : Recursively called backtracking algorithm finding a set of valid paths for the robots to follow
    * build_path : Calls solve for a given list of start and end Points. Handles logging and formatting return data
    * pack_to_points : Convert parallel lists of (x,y) destination (and optionally origin) paris to Point class
    * add_to_cache : Adds destinations in a valid order (according to the solve method) to the .json cache, or creates a
       new cache when no cache is found
    * check_cache : Search .json cache for the given phrase and return the entry if found

    Class
    -----
    * Point : A simple class for representing a 2-D point
    * Line : A class representing a line segment
"""
import json
# import matplotlib.pyplot as plt
import os
import random
import time
import copy
import usafalog

logger = usafalog.CreateLogger(__name__)

cache_filename = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/PathCache.json"


class Point:
    """
    A simple class for representing a 2-D point

    Attributes
    ----------
    x : x position
    y : y position
    """
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self):
        return f"x: {self.x} y: {self.y}"


# Given two points, defines a line
class Line:
    """
    A class representing a line segment

    Attributes
    ----------
    m : float
        slope of the line
    b : float
        y-intercept
    left : float
        the left most (min) x-value
    right : float
        the rightmost (max) x-value
    bottom : float
        the lowest (min) y-value
    top : float
        the highest (max) y-value
    start : Point
        beginning of line segment
    end : Point
        end of line segment
    """
    def __init__(self, start=Point(), end=Point()):
        """
        Parameters
        ----------
        start : Point, optional
            If not provided creates a point at the origin
        end : Point, optional
            If not provided creates a point at the origin
        """
        # Use the point-slope formula to calculate the slope
        try:
            self.m = (end.y - start.y) / (end.x - start.x)
        except ZeroDivisionError:  # vertical line
            self.m = float('inf')
        self.b = start.y - self.m * start.x
        self.left = min(start.x, end.x)
        self.right = max(start.x, end.x)
        self.bottom = min(start.y, end.y)
        self.top = max(start.y, end.y)
        self.start = start
        self.end = end


# Minimum clearance between line and robot
BUFFER_DIST = .5  # meters

# Default starting Points
x_robot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.4, 2.6, 2.8, 3.2, 3.4, 3.6, 3.8, 6.0, 6.0, 6.0, 6.0, 3.6, 3.4, 3.2,
           2.8, 2.6, 2.4]
y_robot = [2.2, 2.4, 2.6, 2.8, 3.2, 3.4, 3.6, 3.8, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 3.4, 3.2, 2.8, 2.6, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0]

# Destination Points (DFEC)
x_dest = [2.30, 2.00, 1.60, 1.30, 1.30, 1.00, 1.00, 1.00, 2.00, 2.30, 2.60, 3.00, 3.30, 3.60, 4.00, 4.00, 4.00, 4.60,
          4.60, 3.60, 3.30, 3.30, 3.00, 3.00, 2.00]

y_dest = [2.50, 2.50, 2.50, 2.75, 2.25, 2.00, 2.50, 3.00, 3.00, 3.00, 3.00, 3.00, 3.00, 3.00, 3.00, 2.00, 2.50, 2.00,
          3.00, 2.00, 2.50, 2.00, 2.50, 2.00, 2.00]

# store list of destination points in order
end_points = []

# keep track of number of lines tested
NUM_TRIES = 0


def lines_intersect(line1: Line, line2: Line):
    """
    Determines if and where two line segments intersect

    Returns
    --------
    boolean if lines intersect
    x-value of intersection (dummy passed in case of no intersection)
    """
    # Determine where lines intersect without line segment bounds
    try:
        x = (line1.b - line2.b) / (line2.m - line1.m)
    except ZeroDivisionError:
        return False, 0  # parallel lines
    # Check if the intersection occurs within either segment
    if min(line1.left, line2.left) < x < max(line1.right, line2.right):
        return True, x
    else:
        return False, x


def line_close_to_point(line: Line, point: Point, endpoints=False):
    """
    Determine if a line segment is ever closer than the BUFFER_DIST to a point

    Creates a line through the given point perpendicular to the given line

    Parameters
    ----------
    line : Line
        line segment to be checked
    point : Point
        point to be checked
    endpoints : bool, optional
        specifies whether endpoint distances should be considered when checking all distances

    Returns
    --------
    boolean true if line is within BUFFER_DIST of point, otherwise false
    """
    # Find line that is perpendicular to the given line and runs through given point
    perpendicular_line = Line()
    try:
        perpendicular_line.m = -1 / line.m
    except ZeroDivisionError:
        perpendicular_line.m = float('Inf')
    perpendicular_line.b = point.y - perpendicular_line.m * point.x
    # find where the shortest distance to point occurs
    intersect, x = lines_intersect(line, perpendicular_line)
    y = line.m * x + line.b
    # if intersection occurs in the line segment
    if line.left < x < line.right:
        dist = ((point.x - x) ** 2 + (point.y - y) ** 2) ** .5
        return dist < BUFFER_DIST
    # if intersection is not on line segment, but considering endpoint distances
    elif endpoints:
        dist = min(((point.x - line.start.x) ** 2 + (point.y - line.start.y) ** 2) ** .5,
                   ((point.x - line.end.x) ** 2 + (point.y - line.end.y) ** 2) ** .5)
        return dist < BUFFER_DIST
    # Default/error case
    else:
        return False


def animate(line: Line, plot_interval: float):
    """
    Creates a pyplot to show proposed line and all established endpoints

    Matplotlib disabled to protect against possible TKINTER errors, but can be re-enabled to use this function, mainly
     for debugging

    Parameters
    ----------
    line : Line
        Current proposed line
    plot_interval : float
        Minimum time to wait before exiting function (min time to wait until shows the next plot)
    """
    plt.close('all')
    fig, ax = plt.subplots()  # Needed for the circles
    # Plot all end points
    for point in end_points:
        cir = plt.Circle((point.x, point.y), BUFFER_DIST, color='r', fill=False)
        ax.plot(point.x, point.y, 'k^')
        ax.add_patch(cir)
    plt.plot([line.start.x, line.end.x], [line.start.y, line.end.y])
    plt.plot(line.end.x, line.end.y, "g*")
    plt.axis([0, 6, 0, 6])
    plt.xlabel("East-West Axis of Robot Workspace (m)")
    plt.ylabel("North-South Axis of Robot Workspace (m)")
    plt.show()
    tic = time.perf_counter()
    while time.perf_counter() - tic < plot_interval:
        pass


def is_valid_line(line: Line):
    """Compares line to all established endpoints and determines if the path is valid to be taken by the usafabot

    A path is valid iff it does not come too close to any points already established

    Returns
    --------
    boolean if line is valid
    """
    for point in end_points:  # Check does not cross any set points
        if line_close_to_point(line, point):
            return False
    return True  # return True if no points or if no conflicts


def solve(origins: list, destinations: list):
    """
    Recursively called backtracking algorithm finding a set of valid paths for the robots to follow

    Parameters
    ----------
    origins : list of Points
        list of robot starting locations
    destinations : list of Points
        list of robots destination locations

    Returns
    --------
    boolean representing of if the algorithm was able to find a valid path for the current starting position
    """
    # Debugging information keeping track of how many paths are attempted
    global NUM_TRIES
    NUM_TRIES += 1
    if destinations:  # if not all of the destination positions have been assigned
        start = origins.pop(0)  # get current start position
        for i, dest in enumerate(destinations):  # loop through all of the remaining positions
            test_line = Line(start, dest)  # draw a line between the start and end
            # animate(test_line, .2)
            if is_valid_line(test_line):  # if the line isn't too close to other established end points
                end_points.append(dest)  # add the end point to the list of established end points
                destinations.pop(i)  # remove the end point from the list of possible end points
                if solve(origins, destinations):  # if the levels below work, return True
                    return True
                end_points.pop()  # Doesn't work at lower levels, so remove dest from list of established points
                destinations.insert(i, dest)  # Doesn't work at lower levels, so add dest back to list in original spot
        origins.insert(0, start)  # return start position to list of all possible start positions in its original place
        return False  # could not find a dest that worked
    return True  # all starting positions have been assigned, so it worked


def build_path(starts: list, ends: list):
    """
    Calls solve for a given list of start and end Points. Handles logging and formatting return data

    Parameters
    ----------
    starts : list of Points
        All starting locations of robots
    ends : list of Points
        All destination points

    Returns
    --------
    return_x : list
        list of x locations in order of the starts list
    return_y : list
        list of y locations in order of the starts list
    """
    global NUM_TRIES
    global end_points
    end_points = []
    NUM_TRIES = 0
    return_x = []
    return_y = []
    plot_copy = copy.copy(starts)
    if solve(starts, ends):  # Successful run of solve will fill endpoints with destinations in a valid order
        logger.debug(f"{NUM_TRIES} paths were tried")
        logger.debug("Destinations:")
        # Unpack each endpoint into an x and y component
        for point in end_points:
            return_x.append(point.x)
            return_y.append(point.y)
            logger.info(point.__str__())
        # plot_result(plot_copy, return_x, return_y)
    else:
        logger.warning(f"unable to solve (tried {NUM_TRIES} times)")
    # log any remaining points
    if starts:  # Should be false for any successful run
        logger.info(f"{len(starts)} remaining starting positions")
    if ends:
        logger.info(f"{len(ends)} remaining destination positions")

    return return_x, return_y


def pack_to_points(x_end: list, y_end: list, x_start=None, y_start=None):
    """
    Convert parallel lists of (x,y) destination (and optionally origin) paris to Point class

    Parameters
    ----------
    x_end : list
        list of destination x positions to complement the accompanying y list
    y_end : list
        list of destination y positions to complement the accompanying x list
    x_start : list, optional
        list of starting x positions to complement the accompanying y list.
        If not provided, default positions defined above are used
    y_start : list, optional
        list of starting y positions to complement the accompanying x list.
        If not provided, default positions defined above are used

    Returns
    -------
    starting_points : list of Points
        list of Point objects that contain the starting positions of the robots with their order persevered
    ending_points : list of Points
        list of Point objects that contain the destination positions of the robots with their order persevered
    """
    # Handle default variables
    if y_start is None:
        y_start = y_robot
    if x_start is None:
        x_start = x_robot
    # pack into list of tuples
    start_list = list(zip(x_start, y_start))
    dest_list = list(zip(x_end, y_end))
    # convert to list of points
    starting_points = []
    ending_points = []
    for i in range(0, len(start_list)):
        starting_points.append(Point(start_list[i][0], start_list[i][1]))
    for i in range(0, len(dest_list)):
        ending_points.append(Point(dest_list[i][0], dest_list[i][1]))
    return starting_points, ending_points


def add_to_cache(phrase: str, x: list, y: list):
    """
    Adds destinations in a valid order (according to the solve method) to the .json cache, or creates a new cache when
     no cache is found

    Reads the .json file as a dictionary and adds/updates the entry with the parameter 'phrase' as a reference. Writes
     the dictionary back to the .json file

    Parameters
    ----------
    phrase : str
        the phrase being spelled out by the destination positions
    x : list
        list of x coordinates for the successful spelling of the phrase
    y : list
        list of y coordinates for the successful spelling of the phrase
    """
    cache_dict = {}

    try:
        f = open(cache_filename)
        cache_dict = json.load(f)
        f.close()
    except OSError:
        logger.warning("JSON cache not found")
        pass

    f = open(cache_filename, 'w')
    cache_dict[phrase] = [x, y]
    json.dump(cache_dict, f, indent=4)
    f.close()


def check_cache(phrase):
    """
    Search .json cache for the given phrase and return the entry if found

    Returns
    --------
    If entry found, 2 parallel lists of x,y coordinates for the destinations of the phrase
    Otherwise returns None
    """
    cache_dict = {}
    try:
        f = open(cache_filename)
        cache_dict = json.load(f)
        f.close()
    except OSError:
        pass

    return cache_dict.get(phrase, None)


if __name__ == '__main__':
    # try multiple times/shuffles
    # plt.close('all')
    robot_starts, robot_ends = pack_to_points(x_dest, y_dest, x_robot, y_robot)
    # For testing
    random.shuffle(robot_ends)
    build_path(robot_starts, robot_ends)
