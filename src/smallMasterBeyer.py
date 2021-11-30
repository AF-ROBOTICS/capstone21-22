#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground and Air Robot Teaming Capstone: Ground Controller
Date: 10 Feb 2020
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

# TODO: take current robot position instead of assigning initial positions - fixed
# TODO: what if you have more or less robots than needed?

# Global Variables
# robotDestination = []
DEST_DIST = .25
# TODO: read  11
# TODO: change this list to match number of robots
# must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2']
#x_dest = [1.0, 1.0, 1.0, 1.30, 1.30, 1.60, 2.0, 2.0, 2.0, 2.3, 2.3, 2.6, 3.0, 3.0, 3.0, 3.3, 3.3, 3.3, 3.6, 3.6, 4.0, 4.0, 4.0, 4.6, 4.6]
#y_dest = [3.0, 2.5, 2.0, 2.75, 2.25, 2.5, 3.0, 2.5, 2.0, 3.0, 2.5, 3.0, 3.0, 2.5, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0]

x_dest = [1,2,3]
y_dest = [1,2,3]

# Define the Controller class
class Master:

    def __init__(self, USAFABOT):
        self.curr_pos = Pose()
        self.dest_pos = Point()
        self.name = USAFABOT

        # -----------------------------------------------------------------------------
        # Topics and Timers
        # -----------------------------------------------------------------------------
        # Publish to the controller
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size=10)
        #rospy.Timer(rospy.Duration(.1), self.callback_Pos)

        # Listen for the bots' current position to the controller
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)
        # rospy.Subscriber(TIBot + '/odom', Odometry, self.callback_groundListener)

    # -------------------------------------------------------------------------------
    # Class Functions
    # -------------------------------------------------------------------------------
    def setGroundDestPosition(self, x, y):
        # Data based on drone position
        self.dest_pos.x = x
        self.dest_pos.y = y

    def callback_currPos(self, data):
        self.curr_pos.position.x = round(data.position.x, 3)
        self.curr_pos.position.y = round(data.position.y, 3)
        self.curr_pos.orientation.z = round(data.orientation.z, 3)
        
    def callback_groundListener(self, data):
        self.curr_pos.position.x = data.position.x
        self.curr_pos.position.y = data.position.y
        self.curr_pos.orientation.z = data.orientation.z
        # rospy.loginfo(rospy.get_caller_id() + 'I heard position:%s' %(self.gnd_curr))

        
    def getCurrPos(self):
        return self.curr_pos.position.x, self.curr_pos.position.y;

    def setDestPosition(self, x, y):
        self.dest_pos.x = x
        self.dest_pos.y = y 


bots = []

if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # Assign number of robot masters
    for k in robots:
        bots.append(Master(k))
        
    # Global Variables
    xrobot = []
    yrobot = []

#    Get initial bot positions
    for bot in bots:
        x = 0
        y = 0
        # block until bot's RR is operational
        print("Waiting bot: " + bot.name)
        tic = time.perf_counter()
        while x == 0 and y == 0:
           x, y = bot.getCurrPos()
           #if (time.perf_counter() - tic) > 5:
           #     print("timeout, " + bot.name)
           #     break;
        xrobot.append(x)
        yrobot.append(y)
        toc = time.perf_counter()
        t = toc-tic
        print("Completed bot: " + bot.name)
        print(t)

    print(xrobot)
    print(yrobot)

#    print("Robot Init X = ", xrobot)
#    print("Robot Init Y = ", yrobot)
#    print("Combined RobotDestination = ", x_dest, y_dest)

    # Assign final bot destinations
    i = 0

    for bot in bots:
        bot.setDestPosition(x_dest[i], y_dest[i])
        bot.pub.publish(bot.dest_pos)
        print("Dest set for:" + bot.name)
        i+=1
#        tic = time.perf_counter()
#        curr_x, curr_y = bot.getCurrPos()
#        init_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5

#        while True:
#            bot.setGroundDestPosition(x_dest[i], y_dest[i])
#            bot.pub.publish(bot.dest_pos)
#            curr_x, curr_y = bot.getCurrPos()
#            print(curr_x, curr_y)
#            curr_dist = ((x_dest[i] - bot.curr_pos.position.x) ** 2 + (y_dest[i] - self.curr_pos.position.y) ** 2) ** 0.5
#            if curr_dist < DEST_DIST :
#                i += 1
#                toc = time.perf_counter()
#                print(bot.name + " is complete")
#                print(toc-tic)
#                bot.setGroundDestPosition(0, 0)
#                break
                
                
    print("all bots complete")
    rospy.spin()
