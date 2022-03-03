#!/usr/bin/env python3

# Import important libraries
import rospy
import time
import math
import random
import numpy as np
from hungarian import build_hungarian

from geometry_msgs.msg import Point, Pose

# Must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']

#Destination Points
x_points = [0, 0.22, 0.44, 0.66, 0.88, 1.1, 1.32, 1.54, 1.76, 1.98, 2.2, 2.42, 2.64, 2.86, 3.08, 3.3, 3.52, 3.74, 3.96, 4.18, 4.4, 4.62, 4.84, 5.06, 5.28]
y_points = [0, 0.21, 0.42, 0.63, 0.84, 1.05, 1.26, 1.47, 1.68, 1.89, 2.1, 2.31, 2.52, 2.73, 2.94, 3.15, 3.36, 3.57, 3.78, 3.99, 4.2, 4.41, 4.62, 4.83, 5.04]
x_dest = random.sample(x_points, len(x_points))
y_dest = random.sample(y_points, len(y_points))


# Destination Points Case 1
#x_dest = [3, 1, 3, 1, 3, 1, 2, 2]
#y_dest = [3, 1, 1, 3, 2, 2, 1, 3]

# Destination Points Case 2
#x_dest = [2, 2]
#y_dest = [3, 3]

# Destination Points Case 3
#x_dest = [3, 1]
#y_dest = [2, 2]

# Destination Points Case 4
#x_dest = [2, 2]
#y_dest = [1, 3]

# Closest distance tolerance for avoidance
AVOID_TOL = 0.5 # increased avoid tolerance for test demo with smaller DFEC

# Additional Globals
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
    
    def setGroundDestPosition(self, x, y):
        # Data based on drone position
        self.dest_pos.x = x
        self.dest_pos.y = y

    def setDestPosition(self, x, y):
        self.dest_pos.x = x
        self.dest_pos.y = y
     
    #NEW CONFIG right turn
    def turnRight(self, bots, closest_x, closest_y, i): #ALT turnRight
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
            
    #NEW CONFIG left turn
    def turnLeft(self, bots, closest_x, closest_y, i):
    	if(closest_x >= bots[i].curr_pos.position.x and closest_y >= bots[i].curr_pos.position.y):
    		x_temp[i] = bots[i].curr_pos.position.x - 1000*(closest_y - bots[i].curr_pos.position.y)
    		y_temp[i] = bots[i].curr_pos.position.y + 1750*(closest_x - bots[i].curr_pos.position.x)
    	if(closest_x <= bots[i].curr_pos.position.x and closest_y >= bots[i].curr_pos.position.y):
    		x_temp[i] = bots[i].curr_pos.position.x - 1000*(closest_y - bots[i].curr_pos.position.y)
    		y_temp[i] = bots[i].curr_pos.position.y - 750*(bots[i].curr_pos.position.x - closest_x)
    	if(closest_x >= bots[i].curr_pos.position.x and closest_y <= bots[i].curr_pos.position.y):
    		x_temp[i] = bots[i].curr_pos.position.x + 1000*(bots[i].curr_pos.position.y - closest_y)
    		y_temp[i] = bots[i].curr_pos.position.y + 750*(closest_x - bots[i].curr_pos.position.x)
    	if(closest_x <= bots[i].curr_pos.position.x and closest_y <= bots[i].curr_pos.position.y):
    		x_temp[i] = bots[i].curr_pos.position.x + 1000*(bots[i].curr_pos.position.y - closest_y)
    		y_temp[i] = bots[i].curr_pos.position.y - 1750*(bots[i].curr_pos.position.x - closest_x)
       		
    def airplane(self):
    	
    	#NEW CONFIG: This config solves the problem cases because it turns off the closest robot. A problem may be the computational load when we move to more robots because all the distances are calculated for each robot. TODO: Solve with a live array that doesnt have to be calculated in the airplane function.
    	for i in range(0, len(robots)):
    	    min_dist = 10
    	    e_turn = 0
    	    for j in range(0, len(robots)):
    	    	if(abs(bots[i].curr_pos.position.x-x_dest[i]) <= 0.05 and abs(bots[i].curr_pos.position.y-y_dest[i]) <= 0.05):
    	    	    break
    	    	if (i != j):
    	            dist = math.sqrt((bots[j].curr_pos.position.x- bots[i].curr_pos.position.x)**2 + (bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)**2)
    	            if(dist<=0.35):
    	                print("emergency turn!")
    	                closest_x = bots[j].curr_pos.position.x
    	                closest_y = bots[j].curr_pos.position.y
    	                e_turn=1
    	                bot.turnRight(bots, closest_x, closest_y, i)
    	                break
    	            elif(dist<=min_dist):
    	                min_dist = dist
    	                closest_x = bots[j].curr_pos.position.x
    	                closest_y = bots[j].curr_pos.position.y
    	    if(e_turn==1):
    	        break
    	    elif(min_dist < AVOID_TOL):
    	        if ((x_temp[i]-bots[i].curr_pos.position.x)==0):
    	            slope = (y_temp[i]-bots[i].curr_pos.position.y)/(((x_temp[i])-bots[i].curr_pos.position.x)+0.01)
    	        else:
    	            slope = (y_temp[i]-bots[i].curr_pos.position.y)/(x_temp[i]-bots[i].curr_pos.position.x)
    	        
    	        y_int = y_temp[i]-(slope*x_temp[i])
    	        
    	        if ((x_temp[i]-bots[i].curr_pos.position.x)==0):
    	            slope = (y_temp[i]-bots[i].curr_pos.position.y)/((x_temp[i]+1)-bots[i].curr_pos.position.x)
    	        else:
    	            slope = (y_temp[i]-bots[i].curr_pos.position.y)/(x_temp[i]-bots[i].curr_pos.position.x)
    	        
    	        y_int = y_temp[i]-(slope*x_temp[i])
    	        
    	        if((x_temp[i] >= bots[i].curr_pos.position.x and y_temp[i] >= bots[i].curr_pos.position.y) or (x_temp[i] >= bots[i].curr_pos.position.x and y_temp[i] <= bots[i].curr_pos.position.y)): # CASE 1: dest is above and right
    	            if(closest_y > (closest_x*slope + y_int)): # bot[j] is above line between dest and bot[i], turn RIGHT
    	                bot.turnRight(bots, closest_x, closest_y, i)
    	            else: # bot[j] is below line between dest and bot[i], turn LEFT
    	                bot.turnLeft(bots, closest_x, closest_y, i)
    	        if((x_temp[i] <= bots[i].curr_pos.position.x and y_temp[i] >= bots[i].curr_pos.position.y) or (x_temp[i] <= bots[i].curr_pos.position.x and y_temp[i] <= bots[i].curr_pos.position.y)): # CASE 2: dest is above and left
    	            if(closest_y < (closest_x*slope + y_int)): # bot[j] is below line between dest and bot[i], turn RIGHT
    	                bot.turnRight(bots, closest_x, closest_y, i)
    	            else: # bot[j] is above line between dest and bot[i], turn LEFT
    	                bot.turnLeft(bots, closest_x, closest_y, i)
		
    	        #bot.turnRight(bots, closest_x, closest_y, i)
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
    
    x_temp = x_dest.copy()
    y_temp = y_dest.copy()

    # Publish new temporary desination positions. Then, run airplane function for basic collision avoidance with updated current positions.
    while 1==1:
    	   		
    	for k in range(0, len(robots)):
    		bots[k].setDestPosition(x_temp[k], y_temp[k])
    	for bot in bots:
    		bot.pub.publish(bot.dest_pos)
    	
    	bot.airplane()
    		
    	#bot.airplane()
	#rospy.spin()
    
