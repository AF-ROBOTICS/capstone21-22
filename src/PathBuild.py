import json
import matplotlib.pyplot as plt
import os
import random
import time

import usafalog

logger = usafalog.CreateLogger(__name__)

cache_filename = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/PathCache.json"


class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self):
        return f"x: {self.x} y: {self.y}"


# Given two points, defines a line
class Line:
    def __init__(self, start=Point(), end=Point()):
        try:
            self.m = (end.y - start.y) / (end.x - start.x)
        except ZeroDivisionError:
            self.m = float('inf')
        self.b = start.y - self.m * start.x
        self.left = min(start.x, end.x)
        self.right = max(start.x, end.x)
        self.bottom = min(start.y, end.y)
        self.top = max(start.y, end.y)
        self.start = start
        self.end = end


# Minimum clearance between line and robot
BUFFER_DIST = .2  # meters

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


def lines_intersect(line1, line2):
    try:
        x = (line1.b - line2.b) / (line2.m - line1.m)
    except ZeroDivisionError:
        return False  # parallel lines
    if min(line1.left, line2.left) < x < max(line1.right, line2.right):
        return True, x
    else:
        return False, x


def line_close_to_point(line: Line, point: Point, endpoints=False):
    # Find perpendicular line through given point
    perpendicular_line = Line()
    perpendicular_line.m = -1 / line.m
    perpendicular_line.b = point.y - perpendicular_line.m * point.x
    # find where the shortest distance to point occurs
    intersect, x = lines_intersect(line, perpendicular_line)
    y = line.m * x + line.b
    # Shortest distance from travel line to closet point

    if line.left < x < line.right:
        dist = ((point.x - x) ** 2 + (point.y - y) ** 2) ** .5
        if dist < BUFFER_DIST:
            return True
        else:
            return False
    elif endpoints:
        dist = min(((point.x - line.start.x) ** 2 + (point.y - line.start.y) ** 2) ** .5,
                   ((point.x - line.end.x) ** 2 + (point.y - line.end.y) ** 2) ** .5)
        if dist < BUFFER_DIST:
            return True
        else:
            return False
    else:
        return False


# path is valid iff it does not come too close to any points already established
def animate(line, plot_interval):
    plt.close('all')
    for point in end_points:
        plt.plot(point.x, point.y, 'r^')
    plt.plot([line.start.x, line.end.x], [line.start.y, line.end.y])
    plt.plot(line.end.x, line.end.y, "g*")
    plt.axis([0, 6, 0, 6])
    plt.show()
    tic = time.perf_counter()
    while time.perf_counter() - tic < plot_interval:
        pass


def is_valid_line(line):
    for point in end_points:  # Check does not cross any set points
        if line_close_to_point(line, point):
            return False
    return True  # return True if no points or if no conflicts


# Recursively called function
def solve(origins, destinations):
    global NUM_TRIES
    NUM_TRIES += 1
    if destinations:  # if not all of the destination positions have been assigned
        start = origins.pop(0)  # get current start position
        for i, dest in enumerate(destinations):  # loop through all of the remaining positions
            test_line = Line(start, dest)  # draw a line between the start and end
            # animate(test_line, .2)
            if is_valid_line(test_line):  # if the line isn't too close to other established end points
                end_points.append(dest)
                destinations.pop(i)  # remove the end point from the list of possible end points
                if solve(origins, destinations):  # if the levels below work, return True
                    return True
                end_points.pop()  # Doesn't work at lower levels so remove dest from established points
                destinations.insert(i, dest)  # Doesn't work below so add dest back to list in original spot
        origins.insert(0, start)
        return False  # could not find a dest that worked
    return True  # all starting positions have been assigned, so it worked


def build_path(starts, ends):
    global NUM_TRIES
    global end_points
    end_points = []
    NUM_TRIES = 0
    return_x = []
    return_y = []
    if solve(starts, ends):
        logger.debug(f"{NUM_TRIES} paths were tried")
        logger.debug("Destinations:")
        for point in end_points:
            return_x.append(point.x)
            return_y.append(point.y)
            logger.info(point.__str__())
        plot_result(return_x, return_y)
    else:
        logger.warning(f"unable to solve (tried {NUM_TRIES} times)")
    # log any remaining points
    if starts:
        logger.info(f"{len(starts)} remaining starting positions")
    if ends:
        logger.info(f"{len(ends)} remaining destination positions")

    return return_x, return_y


def plot_result(xpoints, ypoints):
    for i in range(0, len(xpoints)):
        plt.plot([x_robot[i], xpoints[i]], [y_robot[i], ypoints[i]])
    plt.axes([0, 6, 0, 6])
    plt.show()
    plt.axes([0, 6, 0, 6])
    plt.plot(xpoints, ypoints, 'r*')
    plt.show()


def pack_to_points(x_end, y_end, x_start=None, y_start=None):
    # pack into list of tuples
    if y_start is None:
        y_start = y_robot
    if x_start is None:
        x_start = x_robot
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


def add_to_cache(phrase, x, y):
    cache_dict = {}

    try:
        f = open(cache_filename)
        cache_dict = json.load(f)
        f.close()
    except OSError:
        pass

    f = open(cache_filename, 'w')
    cache_dict[phrase] = [x, y]
    json.dump(cache_dict, f, indent=4)
    f.close()


def check_cache(phrase):
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
    plt.close('all')
    robot_starts, robot_ends = pack_to_points(x_dest, y_dest, x_robot, y_robot)
    # For testing
    random.shuffle(robot_ends)
    build_path(robot_starts, robot_ends)
