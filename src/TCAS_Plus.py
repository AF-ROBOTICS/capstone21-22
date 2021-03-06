#!/usr/bin/env python3

# Import important libraries
import rospy
import time
import math
import random
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
	x_dest[i]=1.3*x_dest[i]-1.1	# this size works well in grid simulation
	y_dest[i]=1.5*y_dest[i]-2	# this size works well in grid simulation

temp_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
temp_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Closest distance tolerance for avoidance
#AVOID_TOL = 0.249 # this is the ideal distance for robots to go in between each other for the original dest points
#AVOID_TOL = 0.3 # increased avoid tolerance for test demo with smaller DFEC
AVOID_TOL = 0.35 # increased avoid tolerance for test demo with smaller DFEC
#AVOID_TOL = 0.4 # increased avoid tolerance for test demo with larger DFEC
BreakJ = False

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
        
    def turnRight(self, bots, temp_x, temp_y, i, j):
    	if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y - 500*(bots[j].curr_pos.position.x-bots[i].curr_pos.position.x)
    	if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y + 2000*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
    	if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y - 2000*(bots[j].curr_pos.position.x - bots[i].curr_pos.position.x)
    	if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y + 500*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)    
    
    def turnLeft(self, bots, temp_x, temp_y, i, j):
    	if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y + 2000*(bots[j].curr_pos.position.x-bots[i].curr_pos.position.x)
    	if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y - 500*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
    	if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y + 500*(bots[j].curr_pos.position.x - bots[i].curr_pos.position.x)
    	if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y):
    		temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    		temp_y[i] = bots[i].curr_pos.position.y - 2000*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
  
    # Iterates through bots current positions and if they are too close to one another, 
    # they will temporarly alter their path by making a right turn to avoid collision.
    def airplane(self):
    	# bots[i].curr_pos.position.x/y is most current position, temp_x/y is the most current destination locations, and dest_x/y are the invariable final locations.
    	# POTENTIAL UPDATE: We stop checking bots if only one is close, and start turning immediatly. An update would be to turn based on the closest robot.
    	for i in range(0, len(robots)): # the i robot is the robot we will be manipulating
    		print("Now I will check bots around: " + bots[i].name)
    		for j in range(0, len(robots)): # the j robots are the bots around the main i robot we are manipulating
    			BreakJ = False # Reset BreakJ variable
    			if (i != j): # skip self
    				if(abs(bots[i].curr_pos.position.x-x_dest[i]) <= 0.05 and abs(bots[i].curr_pos.position.y-y_dest[i]) <= 0.05):
    					print("I'm at my final spot so I'm not moving! " + bots[i].name)
    					BreakJ=True
    				if(BreakJ==False): # if robot is still moving towards it's final desination...
    					dist = math.sqrt((bots[j].curr_pos.position.x- bots[i].curr_pos.position.x)**2 + (bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)**2)
    					if(dist < AVOID_TOL):
    						print("There is a bot too close to: " + bots[i].name)
    						print("The bot that is too close is: " + bots[j].name)
    						print("Since there is a bot that is too close, I'll turn.")
    						
    						
    						
    						#TODO: Add another loop here that starts at the conflict j and sees if there are more robots around it. if there are, we have to do somthing so they dont all get locked into a rotation like we are currently seeing.
    						

    						# Create a line that goes through robot and its destination position. Then determine wheter the conflift is to the right or left of that line
    						
    						#TODO: Robots occasionally get caught in a turning lock when they are on the same x plane. Fix this by either making turns less than 90 or variable turning bases on location of collision.
    						#TODO: May need to have temp instead of y_dest. Using dest may be a problem when it is turning
    						
    						if ((x_dest[i]-bots[i].curr_pos.position.x)==0):
    							slope = (y_dest[i]-bots[i].curr_pos.position.y)/((x_dest[i]+1)-bots[i].curr_pos.position.x)
    						else:
    							slope = (y_dest[i]-bots[i].curr_pos.position.y)/(x_dest[i]-bots[i].curr_pos.position.x)
    						
    						y_int = y_dest[i]-(slope*x_dest[i])
    						
    						#slope_between_collision_and_current= (bots[j].curr_pos.position.y-bots[i].curr_pos.position.y)/(bots[j].curr_pos.position.x-bots[i].curr_pos.position.x)
    						#y_int_of_line_between_collision_and_current= bots[j].curr_pos.position.y-(slope_between_collision_and_current*bots[j].curr_pos.position.x)
					
    						#NEW TEST: LEFT and RIGHT TURNS: Made four new cases based on line between dest position and curr_x/y) 
    						#				  (then determine whether j robot is above or below the line that includes the dest and i robot.)
    						if(x_dest[i] >= bots[i].curr_pos.position.x and y_dest[i] >= bots[i].curr_pos.position.y): # CASE 1: dest is above and right
    							if(bots[j].curr_pos.position.y > (bots[j].curr_pos.position.x*slope + y_int)): # bot[j] is above line between dest and bot[i], turn RIGHT
    								bot.turnRight(bots, temp_x, temp_y, i, j)
    							else: # bot[j] is below line between dest and bot[i], turn LEFT
    								bot.turnLeft(bots, temp_x, temp_y, i, j)
    						if(x_dest[i] <= bots[i].curr_pos.position.x and y_dest[i] >= bots[i].curr_pos.position.y): # CASE 2: dest is above and left
    							if(bots[j].curr_pos.position.y < (bots[j].curr_pos.position.x*slope + y_int)): # bot[j] is below line between dest and bot[i], turn RIGHT
    								bot.turnRight(bots, temp_x, temp_y, i, j)
    							else: # bot[j] is above line between dest and bot[i], turn LEFT
    								bot.turnLeft(bots, temp_x, temp_y, i, j)
    						if(x_dest[i] >= bots[i].curr_pos.position.x and y_dest[i] <= bots[i].curr_pos.position.y): # CASE 3: dest is below and right
    							if(bots[j].curr_pos.position.y > (bots[j].curr_pos.position.x*slope + y_int)): # bot[j] is above line between dest and bot[i], turn RIGHT
    								bot.turnRight(bots, temp_x, temp_y, i, j)
    							else: # bot[j] is below line between dest and bot[i], turn LEFT
    								bot.turnLeft(bots, temp_x, temp_y, i, j)
    						if(x_dest[i] <= bots[i].curr_pos.position.x and y_dest[i] <= bots[i].curr_pos.position.y): # CASE 4: dest is below and left
    							if(bots[j].curr_pos.position.y < (bots[j].curr_pos.position.x*slope + y_int)): # bot[j] is below line between dest and bot[i], turn RIGHT
    								bot.turnRight(bots, temp_x, temp_y, i, j)
    							else: # bot[j] is above line between dest and bot[i], turn LEFT
    								bot.turnLeft(bots, temp_x, temp_y, i, j)
    						
    						#OLD (Do not change): RIGHT turns only:
    						#if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y): # above and right
    						#	temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y - 1000*(bots[j].curr_pos.position.x-bots[i].curr_pos.position.x)
    						#if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y): # above and left
    						#	temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y + 1000*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
    						#if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y): # below and right
    						#	temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y - 1000*(bots[j].curr_pos.position.x - bots[i].curr_pos.position.x)
    						#if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y): # below and left
    						#	temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y + 1000*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
    							
    						#OLD (Do not change): LEFT turns only:
    						#if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y): # above and right
    						#	temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y + 1000*(bots[j].curr_pos.position.x-bots[i].curr_pos.position.x)
    						#if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y): # above and left
    						#	temp_x[i] = bots[i].curr_pos.position.x - 1000*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y - 1000*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
    						#if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y): # below and right
    						#	temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y + 1000*(bots[j].curr_pos.position.x - bots[i].curr_pos.position.x)
    						#if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y): # below and left
    						#	temp_x[i] = bots[i].curr_pos.position.x + 1000*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    						#	temp_y[i] = bots[i].curr_pos.position.y - 1000*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)		

    						BreakJ=True
    					else:
    						print("continue checking bots around: " + bots[i].name)
    						print("this bot is not too close: " + bots[j].name)
    						print(dist)
    						print("Since there is no one near me, I'll go to my final desination of x_dest and y_dest.")
    						if (j >= 23):
    							temp_x[i] = x_dest[i]
    							temp_y[i] = y_dest[i] 
    						print(x_dest[i])
    						print(y_dest[i])

    			# this needs to be a separate if statement so the correct loop is broken out of. We did not know how to do a double break
    			if (BreakJ == True): # if a j robot is too close to the i robot, we're not even going to keep checking because the i robot needs to get turning ASAP
    				print('Breaking out of J')
    				break
    		print("end J")
    	print("end I")

# create empty array of bots
bots = []

if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # Assign number of robot masters
    for k in robots:
        bots.append(Master(k))
    
    # Global Variables
    xrobot = []
    yrobot = []

    # Get initial bot positions
    for bot in bots:
        x = 0
        y = 0
        # block until bot's RR is operational
        print("Waiting bot: " + bot.name)
        while(x == 0 and y == 0):
            x,y = bot.getCurrPos()
        xrobot.append(x)
        yrobot.append(y)
        print("Completed initial assignment for bot: " + bot.name)
    
    # Puts initial destination and current coordinates into coordList. coordList gives us optimized final locations.
    coordList = build_hungarian(xrobot, yrobot, x_dest, y_dest)
 	   
    # Assign optimized final bot destinations
    for k in range(0, len(robots)):
    	 x_dest[k] = coordList[bots[k].name][0]
    	 y_dest[k] = coordList[bots[k].name][1]
    	 
    # Copy final destination values to temporary desination array. Had to use .copy() because editing temp_x would edit dest_x
    temp_x = x_dest.copy()
    temp_y = y_dest.copy()

    # Publish new temporary desination positions. Then, run airplane function for basic collision avoidance with updated current positions.
    while 1==1:
    
    	#Publish current values
    	for k in range(0, len(robots)):
    		bots[k].setDestPosition(temp_x[k], temp_y[k])
    	for bot in bots:
    		bot.pub.publish(bot.dest_pos)
    	
        # Current positions are bots[i].curr_pos.position.x and bots[i].curr_pos.position.y. We subscribe to this. Run airplane to check for collisions.
    	bot.airplane()
  	
  	# Test Outputs
    	#print(bots[0].curr_pos.position.x)
    	#print(bots[0].curr_pos.position.y)
    	#print(bots[23].dest_pos.x)
    	#print(bots[23].dest_pos.y)
    	#print(temp_x[23])
    	#print(temp_y[23])
    	#print(x_dest[23])
    	#print(y_dest[23])
    	

	#rospy.spin()
    
