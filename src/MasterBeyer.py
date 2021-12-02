#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground and Air Robot Teaming Capstone: Ground Controller
Date: 29 Nov 2021
----------------------------------------------------------------------------------"""

# Import important libraries
import roslib
import rospy
import time
import serial
import array
import signal
from std_msgs.msg import String
from threading import Thread

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# TODO: take current robot position instead of assigning initial positions - fixed
# TODO: what if you have more or less robots than needed?

# Global Variables
xrobot = []
yrobot = []
robotDestination = []
DEST_DIST = .25 #meters
TIMEOUT_THRESH = 10 #seconds


# must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']
#DFEC from inside to outside
x_dest = [2.3,2,1.6,1.3,1.3,1,1,1,2,2.3,2.6,3,3.3,3.6,4,4,4,4.6,4.6,3.6,3.3,3.3,3,3,2]
y_dest = [2.5,2.5,2.5,2.75,2.25,3,2.5,2,3,3,3,3,3,3,3,2,2.5,2,3,2,2.5,2,2.5,2,2]

def handler(signum, frame):
	print("KILLED with CTRL_C")
	exit(1)

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
        
    def getCurrPos(self):
        return self.curr_pos.position.x, self.curr_pos.position.y;

    def setDestPosition(self, x, y):
        self.dest_pos.x = x
        self.dest_pos.y = y 


bots = []
signal.signal(signal.SIGINT, handler)
if __name__ == '__main__':
	rospy.init_node('master', anonymous=True)
    # Assign number of robot masters
	for k in robots:
		bots.append(Master(k))

#    Get initial bot positions
	for bot in bots:
		x = 0
		y = 0
		# block until bot's RR is operational
		print("Waiting bot: " + bot.name)
		tic = time.perf_counter()
		while x == 0 and y == 0:
			x, y = bot.getCurrPos()
			if (time.perf_counter() - tic > TIMEOUT_THRESH):
				print("Timeout: " + bot.name)
				break;
			elif x != 0 or y != 0:		           	 
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
		bot.setGroundDestPosition(x_dest[i], y_dest[i])
		bot.pub.publish(bot.dest_pos)
		print("Dest set for:" + bot.name)
		tic = time.perf_counter()
		curr_x, curr_y = bot.getCurrPos()
        # init_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5
		while True:
			bot.setGroundDestPosition(x_dest[i], y_dest[i])
			bot.pub.publish(bot.dest_pos)
			curr_x, curr_y = bot.getCurrPos()
			bot.setGroundDestPosition(x_dest[i], y_dest[i])
			bot.pub.publish(bot.dest_pos)
           	# print(curr_x, curr_y)
			curr_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5
			if curr_dist < DEST_DIST :
				i += 1
				toc = time.perf_counter()
				print(bot.name + " is complete. It took " + str(round(toc-tic,4)) + " seconds")
				bot.setGroundDestPosition(0, 0) # need this?
				break

                
	print("all bots complete")
	rospy.spin()
