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
# | FILENAME      : Right_Turn_Only.py
# | AUTHOR(S)     : C1C Sean Lewis
# | CREATED       : 11 Jan 2022
# | Last Update   : 06 Apr 2022
"""
This module models the aerospace traffic collision avoidance system (TCAS) with 25 robots. This TCAS system used right turns only.

This script can be ran in both simulation and real world using the command:
'rosrun capstone21-22 Right_Turn_Only.py'
"""

# Import important libraries
import rospy
import time
import math
from hungarian import build_hungarian

from geometry_msgs.msg import Point, Pose

#must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']

#Destination Points

#Original Points:
x_dest = [1.0, 1.0, 1.0, 1.30, 1.30, 1.60, 2.0, 2.0, 2.0, 2.3, 2.3, 2.6, 3.0, 3.0, 3.0, 3.3, 3.3, 3.3, 3.6, 3.6, 4.0, 4.0, 4.0, 4.6, 4.6]
y_dest = [3.0, 2.5, 2.0, 2.75, 2.25, 2.5, 3.0, 2.5, 2.0, 3.0, 2.5, 3.0, 3.0, 2.5, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0]

# Resize DFEC letters
for i in range(0,len(x_dest)):
	x_dest[i]=1.4*x_dest[i]-1.2
	y_dest[i]=3*y_dest[i]-5

x_temp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
y_temp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Closest distance tolerance for avoidance
AVOID_TOL = 0.30 # increased avoid tolerance for test demo with larger DFEC

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
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size = 10)
        # Listen for the bots' current position to the controller
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)
        #rospy.Subscriber(USAFABOT + '/odom', Odometry, self.callback_groundListener)

    #-------------------------------------------------------------------------------
    # Class Functions
    #-------------------------------------------------------------------------------
    def callback_currPos(self, data):
        self.curr_pos.position.x = round(data.position.x, 3)
        self.curr_pos.position.y = round(data.position.y, 3)
        self.curr_pos.orientation.z = round(data.orientation.z, 3)

    def getCurrPos(self):
        return self.curr_pos.position.x, self.curr_pos.position.y;

    def setDestPosition(self, x, y):
        self.dest_pos.x = x
        self.dest_pos.y = y      
        
    # Iterates through bots current positions and if they are too close to one another, 
    # they will temporarly alter their path by making a right turn to avoid collision..
    # TODO make sure the robots do not keep turning once they get to their final destination.
    def airplane(self):
    	for i in range(0, len(robots)): # the i robot is the robot we will be manipulating
    		min_dist = 100
    		skip = 0
    		if(abs(bots[i].curr_pos.position.x-x_dest[i]) <= 0.05 and abs(bots[i].curr_pos.position.y-y_dest[i]) <= 0.05):
    			x_temp[i] = x_dest[i]
    			y_temp[i] = y_dest[i]
    			skip = 1
    		if (skip == 0):
    			for j in range(0, len(robots)):
    				if(i != j):
    					dist = math.sqrt((bots[j].curr_pos.position.x- bots[i].curr_pos.position.x)**2 + (bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)**2)
    					if(dist<=min_dist):
    						min_dist = dist
    						closest_x = bots[j].curr_pos.position.x
    						closest_y = bots[j].curr_pos.position.y
    			if(min_dist < AVOID_TOL):
    			# RIGHT turns:
    				if(closest_x >= bots[i].curr_pos.position.x and closest_y >= bots[i].curr_pos.position.y):
    					x_temp[i] = bots[i].curr_pos.position.x + 1000*(closest_y - bots[i].curr_pos.position.y)
    					y_temp[i] = bots[i].curr_pos.position.y - 750*(closest_x-bots[i].curr_pos.position.x)
    				if(closest_x <= bots[i].curr_pos.position.x and closest_y >= bots[i].curr_pos.position.y):
    					x_temp[i] = bots[i].curr_pos.position.x + 1000*(closest_y - bots[i].curr_pos.position.y)
    					y_temp[i] = bots[i].curr_pos.position.y + 1750*(bots[i].curr_pos.position.x - closest_x)
    				if(closest_x >= bots[i].curr_pos.position.x and closest_y <= bots[i].curr_pos.position.y):
    					x_temp[i] = bots[i].curr_pos.position.x - 1000*(bots[i].curr_pos.position.y - closest_y)
    					y_temp[i] = bots[i].curr_pos.position.y - 1750*(closest_x - bots[i].curr_pos.position.x)
    				if(closest_x <= bots[i].curr_pos.position.x and closest_y <= bots[i].curr_pos.position.y):
    					x_temp[i] = bots[i].curr_pos.position.x - 1000*(bots[i].curr_pos.position.y - closest_y)
    					y_temp[i] = bots[i].curr_pos.position.y + 750*(bots[i].curr_pos.position.x - closest_x)
    			else:
    				x_temp[i] = x_dest[i]
    				y_temp[i] = y_dest[i]

# create empty array of bots
bots = []

if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # Assign number of robot masters
    for k in robots:
        bots.append(Master(k))
    
    # Global Variables
    xrobot = []	# initial x position for first hungarian assignment
    yrobot = []	# initial y position for first hungarian assignment

    # Get initial bot positions
    for bot in bots:
        x = 0
        y = 0
        # block until bot's RR is operational
        while(x == 0 and y == 0):
            x,y = bot.getCurrPos()
        xrobot.append(x)
        yrobot.append(y)
        print("Completed initial assignment for bot: " + bot.name)
    
    # Puts initial destination and current coordinates into coordList. coordList gives us back best final locations.
    coordList = build_hungarian(xrobot, yrobot, x_dest, y_dest)
 	   
    # Assign final bot destinations
    for k in range(0, len(robots)):
    	 x_dest[k] = coordList[bots[k].name][0]
    	 y_dest[k] = coordList[bots[k].name][1]
    	 
    # Uses setDestPosition to assign initial destination positions in coordList to self.dest_pos.x/y and then publishes to bot.dest_pos 
    #for bot in bots:
    #    bot.setDestPosition(coordList[bot.name][0], coordList[bot.name][1])
    #    bot.pub.publish(bot.dest_pos)
 
    x_temp = x_dest.copy()
    y_temp = y_dest.copy()
    
    #for k in range(0, len(robots)):
    #	bots[k].setDestPosition(x_temp[k], y_temp[k])
    
    # Iterate through bots to get current positions and put the positions in the global arrays curr_x and curr_y. Then run airplane function for basic collision avoidance with updated current positions.
    while 1==1:
    
    	#Current
    	for k in range(0, len(robots)):
    		bots[k].setDestPosition(x_temp[k], y_temp[k])
    	for bot in bots:
    		bot.pub.publish(bot.dest_pos)
    	
        # Current positions are bots[i].curr_pos.position.x and bots[i].curr_pos.position.y. We subscribe to this. Run airplane to check for collisions.
    	bot.airplane()
    	
  	# Test Printouts:
    	#print(bots[0].curr_pos.position.x)
    	#print(bots[0].curr_pos.position.y)
    	#print(bots[23].dest_pos.x)
    	#print(bots[23].dest_pos.y)
    	#print(x_temp[23])
    	#print(y_temp[23])
    	#print(x_dest[23])
    	#print(y_dest[23])
    	

	#rospy.spin()
    
