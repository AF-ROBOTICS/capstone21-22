#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground Robot Teaming Capstone: Ground Controller
Date: 15 Nov 2021
----------------------------------------------------------------------------------"""

# Import important libraries
import rospy
import time
import array
from std_msgs.msg import String
from threading import Thread
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import signal

# Import from Other Project Files
from util import CELL_W, Status, Cell
from robot import Robot
from field import Field
from routing import single_greedy
from waterfall import ctrl_c_handler


# TODO: Dynamic Destinations
# TODO: Dynamic Bot Quantity

# Global Variables
init_pos = []
robotDestination = []
DEST_DIST = 0.25
field = Field()


# TODO: change this list to match number of robots
# must match number of robots entered by user
robot_ids = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
    'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
    'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
    'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
    'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']

x_dest = [2.3,2,1.6,1.3,1.3,1,1,1,2,2.3,2.6,3,3.3,3.6,4,4,4,4.6,4.6,3.6,3.3,3.3,3,3,2]
y_dest = [2.5,2.5,2.5,2.75,2.25,3,2.5,2,3,3,3,3,3,3,3,2,2.5,2,3,2,2.5,2,2.5,2,2]

# Test Position of Collapsing
# x_dest = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
# y_dest = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]

# Save the Above Information as an Arrary of Robot Objects
robots = []
for i in range(0, len(robot_ids)):
    robots.append(Robot(robot_ids[i], (x_dest[i], y_dest[i], 0)))

print(robots)

# Enables "ctr + c" handler
signal.signal(signal.SIGINT, ctrl_c_handler)

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

        # Time Performance
        toc = time.perf_counter()
        t = toc-tic
        print("Completed bot: " + robot.name)
        print(t)

    # print("Initial Positions: " + init_pos)

    # Assign final bot destinations
    for i in range(0, len(robots)):
        robots[i].setDest((x_dest[i], y_dest[i]))
        # robots[i].pub.publish(Point(robots[i].dest.x, robots[i].dest.y, 0))
        print("Dest set for:" + robots[i].name)
    
    # Map Destinations in Field
    field.map_dest(robots)
    
    # TODO: while not final formation
    while 1:
        
        # Update Current Field
        field.map_bots(robots)

        # Reset Next Field
        field_next = Field()
        
        # For Every Robot
        for robot in robots:

            # If Robot not Finished
            if not robot.status is Status(2):

                # If Robot was successfully mapped
                if not robot.pos_cell[0] == None and not robot.pos_cell[1] == None:

                    # Get Discrete Dest
                    robot.step_cell = single_greedy(field, field_next, robot)

                    # Convert to Analog
                    x = (robot.step_cell[0] + 0.5) * CELL_W
                    y = (robot.step_cell[1] + 0.5) * CELL_W

                    # Publish Position
                    robot.pub.publish(Point(x, y))
                    print("POINT PUBLISHED!")

                # If Robot not successfully mapped
                # else:
                #     print(robot.ID + " position not available")

            # If Robot Finished
            else:

                # Map Bot to Next Field
                field_next.cells[robot.step_cell[0]][robot.step_cell[1]].robot = robot
        
        # Wait for Robots to Move to New Dest
        time.sleep(10)

    # Finished
    print("All Bots Assembled")
    rospy.spin()
