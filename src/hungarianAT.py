# Import important libraries
import roslib
import rospy
import time
import serial
import array
import numpy as np
import time
from scipy.optimize import linear_sum_assignment
from robot import Robot

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# Robots
robots = ["usafabot0", "usafabot1", "usafabot2", "usafabot3", "usafabot4", "usafabot5", "usafabot6", "usafabot7",
          "usafabot8", "usafabot9", "usafabot10", "usafabot11", "usafabot12", "usafabot13", "usafabot14", "usafabot15",
          "usafabot16", "usafabot17", "usafabot18", "usafabot19", "usafabot20", "usafabot21", "usafabot22",
          "usafabot23", "usafabot24"]

rows = 25
cols = 25
hungarian = np.zeros((rows, cols))
assignments = {}

def build_hungarian(bots, destinations):
    for bot in bots:
        for i in range(1, len(bots)):
            distance = ((bot.curr_pos.x - destinations[i].x)**2 + (bot.curr_pos.y - destinations[i].y)**2) ** 0.5
            distance = round(distance, 2)
            hungarian[i, j] = distance

    row_ind, col_ind = linear_sum_assignment(hungarian)
    print(row_ind)
    print(col_ind)

    for robot, col in zip(robots, col_ind):
        assignments[robot] = [destinations[col].x, destinations[col].y]

    for i in range(1, len(bots)):
        bot.setDest(assignments[i])

    return assignments


if __name__ == '__main__':
    start_time = time.time()
    build_hungarian()
