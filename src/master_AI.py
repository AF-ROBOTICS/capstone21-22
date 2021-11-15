#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground Robot Teaming Capstone: Ground Controller
Date: 15 Nov 2021
----------------------------------------------------------------------------------"""

# Import important libraries
import roslib
import rospy
import time
import serial
import array
from std_msgs.msg import String
from threading import Thread

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from robot import Robot
from field import Field

# TODO: take current robot position instead of assigning initial positions - fixed
# TODO: what if you have more or less robots than needed?

# Global Variables
init_pos = []
robotDestination = []
DEST_DIST = .25

# TODO: change this list to match number of robots
# must match number of robots entered by user
robot_ids = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
    'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
    'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
    'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
    'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']

x_dest = [2.3,2,1.6,1.3,1.3,1,1,1,2,2.3,2.6,3,3.3,3.6,4,4,4,4.6,4.6,3.6,3.3,3.3,3,3,2]
y_dest = [2.5,2.5,2.5,2.75,2.25,3,2.5,2,3,3,3,3,3,3,3,2,2.5,2,3,2,2.5,2,2.5,2,2]

# Save the Above Information as an Arrary of Robot Objects
robots = Robot[None]
for i in range(robot_ids):
    robots.append(Robot(robot_ids[i], (x_dest[i], y_dest[i], 0)))

# Main
if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)

    # Get initial bot positions
    for robot in robots:
        
        # Wait for RR
        print("Waiting bot: " + robot.name)
        tic = time.perf_counter()
        while robot.pos.x == 0 and robot.pos.y == 0:
           time.sleep(0.01)

        init_pos.append((robot.pos.x, robot.pos.y))
        toc = time.perf_counter()
        t = toc-tic
        print("Completed bot: " + robot.name)
        print(t)

    print("Initial Positions: " + init_pos)

    # Assign final bot destinations
    for i in range(robots):
        robots[i].setDest((x_dest[i], y_dest[i]))
        robots[i].pub.publish(robots[i].dest)
        print("Dest set for:" + robots[i].name)
    
    # Finished
    print("All Bots Assembled")
    rospy.spin()
