#!/usr/bin/env python3

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
x_dest = [1.0, 1.0, 1.0, 1.30, 1.30, 1.60, 2.0, 2.0, 2.0, 2.3, 2.3, 2.6, 3.0, 3.0, 3.0, 3.3, 3.3, 3.3, 3.6, 3.6, 4.0, 4.0, 4.0, 4.6, 4.6]
y_dest = [3.0, 2.5, 2.0, 2.75, 2.25, 2.5, 3.0, 2.5, 2.0, 3.0, 2.5, 3.0, 3.0, 2.5, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0]
curr_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
curr_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
temp_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
temp_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Test Destination Points
#x_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#y_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#x_temp_dest = x_dest
#y_temp_dest = y_dest

# closest distance tolerance for avoidance
AVOID_TOL = 0.2 # 0.2 worked

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
    # they will temporarly alter their path to avoid collision. Assumes that all current locations are up to date (getCurrPos is ran before airplane() is called).
    # TODO make sure the robots do not keep turning once they get to their final destination.
    def airplane(self):
    
    # ASSUMES THAT curr_x, curr_y are the most current position and dest_x and dest_y are the invariable final locations
    	# all robot's current locations are in bot.curr_pos.position.x and self.curr_pos.position.y
    	# what I need to find is the closest robots distance and if that distance is smaller than the threshold, have the robot turn right
    	# how do I iterate through all the robots current positions
    	for i in range(0, len(robots)):
    		for j in range(0, len(robots)):
    			if (curr_x[i] != curr_x[j]  and curr_y[i] != curr_y[j]): # if the robot compares itself to itself, it will always be turning
    				dist = math.sqrt((curr_y[i]- curr_y[j])**2 + (curr_x[i] - curr_x[j])**2)
    				print(dist)
    				if (abs(dist) < AVOID_TOL): # if the distance between the i robot and j robot (exculding itself) is small enough, the robot will turn right with the following algorithm
    				# There has to be a better way to do this
    					if(curr_x[i] > x_dest[i]): # moving left
    						#temp_y[i] = y_dest[i] + 2
    						temp_y[i] = y_dest[i] + 1.5**(1/dist)
    						#temp_y[i] = 5 #works
    						
    					#if(x_dest[i] >= curr_x[i] and y_dest[i] >= curr_y[i]): # dest is above and to the right of curr
    					#	temp_x[i] = curr_x + (5*(x_dest-curr_x))
    					#	temp_y[i] = curr_y - (5*(y_dest-curr_y))
    						
    					#if(x_dest[i] >= curr_x[i] and y_dest[i] <= curr_y[i]): # dest is below and to the right of curr
    					#	temp_x[i] = curr_x - (5*(x_dest-curr_x))
    					#	temp_y[i] = curr_y - (5*(y_dest-curr_y))
    						
    					#if(x_dest[i] <= curr_x[i] and y_dest[i] <= curr_y[i]): # dest is below and to the left of curr
    					#	temp_x[i] = curr_x - (5*(x_dest-curr_x))
    					#	temp_y[i] = curr_y + (5*(y_dest-curr_y))
    						
    						
    					#if(x_dest[i] <= curr_x[i] and y_dest[i] >= curr_y[i]): # dest is above and to the left of curr
    					#	temp_x[i] = curr_x + (5*(x_dest-curr_x))
    					#	temp_y[i] = curr_y + (5*(y_dest-curr_y))
    					
    					else: # moving right
    						#temp_y[i] = y_dest[i] - 2
    						temp_y[i] = y_dest[i] - 1.5**(1/dist)
    						#temp_y[i] = 5 #works
    					if(curr_y[i] > y_dest[i]): # moving up
    						#temp_x[i] = x_dest[i] + 2
    						temp_x[i] = x_dest[i] + 1.5**(1/dist)
    						#temp_x[i] = 5 #works
    					else: # moving down
    						#temp_x[i] = x_dest[i] - 2
    						temp_x[i] = x_dest[i] - 1.5**(1/dist)
    						# temp_x[i] = 5 #works
    					break # if there is a single bot next to bot i, it needs to start changing instead of stopping the turn if the next bot is not close.
    				else: # executes when the distance between the i robot and j robot (exculding itself) is no longer critical
    					# set dest position back to its original dest position
    					temp_x[i] = x_dest[i]
    					temp_y[i] = y_dest[i]
    			
    	# publish the update the new destination coordinates
    	coordList = build_hungarian(curr_x, curr_y, temp_x, temp_y)
    	for bot in bots:
    		bot.setDestPosition(coordList[bot.name][0], coordList[bot.name][1])
    		bot.pub.publish(bot.dest_pos)
    		print(bot.dest_pos)

bots = [] 
DEST_TOL = 0.1

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

    # Calculation functions
    
    # Puts initial destination coordinates into coordList
    coordList = build_hungarian(xrobot, yrobot, x_dest, y_dest)
    
    # Assign final bot destinations
    i = 0

    # Uses setDestPosition to assign initial destination positions in coordList to self.dest_pos.x/y and then publishes to bot.dest_pos 
    for bot in bots:
        bot.setDestPosition(coordList[bot.name][0], coordList[bot.name][1])
        bot.pub.publish(bot.dest_pos)
        #print(bot.dest_pos)
        #the_x,the_y = bot.getCurrPos()
        #init_dist = ((x_dest[i] - the_x) ** 2 + (y_dest[i] - the_y) ** 2) ** 0.5 
        #print("running in main")   
    
    # At this point all initial setup is done. xrobot and yrobot are arrays of the initial positions. x_dest and y_dest are arrays of the desination points, not neccesarily ties to a specific robot.
    # curr_x, curr_y, temp_x, and temp_y are all still empty
    # curr_x and curr_y will be used from this point forward for current robot coorinates rather than the xrobot and yrobot.
    
    # Iterate through bots to get current positions and put the positions in the global arrays curr_x and curr_y. Then run airplane function for basic collision avoidance with updated current positions.
    while 1==1:
    	k=0 # Needs to have a k because bots are not number iteratable? There's probobly a better way to do this...
    	for bot in bots:
    		curr_x[k] = bot.curr_pos.position.x
    		print(curr_x)
    		curr_y[k] = bot.curr_pos.position.y
    		print(curr_y)
    		k = k+1
    		
    	# Current positions are now in arrays curr_x and curr_y. Run airplane to check for collisions.
    	bot.airplane()
    	
    	
    	
	#rospy.spin()
    
