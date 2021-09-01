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
import hungarian
from std_msgs.msg import String
from threading import Thread

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# TODO: take current robot position instead of assigning initial positions - fixed
# TODO: what if you have more or less robots than needed? 

# Global Variables
listX = []
listY = []
xrobot = []
yrobot = []
robotDestination = []

# TODO: change this list to match number of robots
#must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']

# Define the Controller class
class Master:

    def __init__(self, USAFABOT):
        self.curr_pos = Pose()
        self.dest_pos = Point()
        self.USAFABOT = USAFABOT

        # -----------------------------------------------------------------------------
        # Topics and Timers 
        # -----------------------------------------------------------------------------      
        # Publish to the controller
        self.pub = rospy.Publisher(USAFABOT + '/dest_pos', Point, queue_size = 10)
        rospy.Timer(rospy.Duration(.1), self.callback_posTalker)
        # Listen for the bots' current position to the controller
        rospy.Subscriber(USAFABOT + '/curr_pos', Pose, self.callback_posListener)
        #rospy.Subscriber(USAFABOT + '/odom', Odometry, self.callback_groundListener)

    #-------------------------------------------------------------------------------
    # Class Functions
    #-------------------------------------------------------------------------------
    def callback_posTalker(self, data):
        # message is printed to the terminal, written to Node log file, and written to rosout
        #rospy.loginfo(self.curr_pos)
        self.pub.publish(self.curr_pos) # work being done
        #rate.sleep()

    def callback_posListener(self, data):
        self.curr_pos.position.x = data.position.x
        self.curr_pos.position.y = data.position.y
        self.curr_pos.orientation.z = data.orientation.z
        #rospy.loginfo(rospy.get_caller_id() + 'I heard position:%s' %(self.gnd_curr))

    def getCurrentPos(self):
        return self.curr_pos.position.x, self.curr_pos.position.y;

    def setDestPosition(self, x, y):
        # Data based on drone position
        self.dest_pos.x = x
        self.dest_pos.y = y


bots = [] #not sure what this guy does


DEST_TOL = 0.1 #also not sure what this guy does

if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # Assign number of robot masters
    for k in robots:
        bots.append(Master(k))

    # Get initial bot positions
    for bot in bots:
        x = 0
        y = 0
        # block until bot's RR is operational
        while(x == 0 and y == 0):
            x,y = bot.getCurrentPos()
        xrobot.append(x)
        yrobot.append(y)
        print("Completed bot: " + bot.USAFABOT)

    # Calculation functions
    coordList = build_hungarian(xrobot, yrobot)



    print("x = ", listX)
    print("y = ", listY)
    print("Comined coordList = ", coordList)

    print("Robot Init X = ", xrobot)
    print("Robot Init Y = ", yrobot)
    print("Combined RobotDestination = ", robotDestination)

    # Assign final bot destinations
    i = 0

    for bot in bots:
        bot.setDestPosition(x_dest[i], y_dest[i])
        curr_x,curr_y = bot.getCurrentPos()
        init_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5        

    rospy.spin()












