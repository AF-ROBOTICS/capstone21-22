#!/usr/bin/env python3

# Import important libraries
import rospy
import time
import math
from hungarianAirplaneTesting import build_hungarian

from geometry_msgs.msg import Point, Pose

#must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4', 'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9', 'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14']
#Destination Points
x_dest = [0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4]
y_dest = [2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4]
#curr_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#curr_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
temp_x = [0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
temp_y = [0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# closest distance tolerance for avoidance
AVOID_TOL = 0.2 # 0.2 worked
COLL_TOL = 0.1
TooClose = False

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
        
    # ASSUMES THAT bots[i].curr_pos.position.x and bots[i].curr_pos.position.y are the most current position, temp_x and temp_y are the most current destination locations, and dest_x and dest_y are the invariable final locations
    	# all robot's current locations are in bot.curr_pos.position.x and self.curr_pos.position.y
    	# what I need to find is the closest robots distance and if that distance is smaller than the threshold, have the robot turn right
    	# how do I iterate through all the robots current positions
    	for i in range(0, len(robots)): # the i robot is the robot we will be manipulating
    		TooClose = False
    		for j in range(0, len(robots)): # the j robots are the bots around the main i robot we are manipulating
    			if (bots[i].curr_pos.position.x != bots[j].curr_pos.position.x and bots[i].curr_pos.position.y != bots[j].curr_pos.position.y): # skip self
    				dist = math.sqrt((bots[i].curr_pos.position.y- bots[j].curr_pos.position.y)**2 + (bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)**2)
    				#print(dist)
    				if (dist < AVOID_TOL): # if the distance between the i robot and j robot (exculding itself) is small enough, the robot will turn right
    					#print(i)
    				
    					# CASE I
    					if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y): # close bot is above and right
    						temp_x[i] = bots[i].curr_pos.position.x + 100*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y) #Multipy because differences will always be very small
    						#if (temp_x[i]<0):
    						#	temp_x[i] = 0
    						#elif (temp_x[i]>6):
    						#	temp_x[i] = 6	
    						temp_y[i] = bots[i].curr_pos.position.y - 100*(bots[j].curr_pos.position.x-bots[i].curr_pos.position.x)
    						#if (temp_y[i]<0):
    						#	temp_y[i] = 0
    						#elif (temp_y[i]>6):
    						#	temp_y[i] = 6
    						
    					# CASE II	
    					if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y >= bots[i].curr_pos.position.y): # close bot is above and left
    						temp_x[i] = bots[i].curr_pos.position.x + 100*(bots[j].curr_pos.position.y - bots[i].curr_pos.position.y)
    						#if (temp_x[i]<0):
    						#	temp_x[i] = 0
    						#elif (temp_x[i]>6):
    						#	temp_x[i] = 6	
    						temp_y[i] = bots[i].curr_pos.position.y + 100*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
    						#if (temp_y[i]<0):
    						#	temp_y[i] = 0
    						#elif (temp_y[i]>6):
    						#	temp_y[i] = 6
    					
    					# CASE III	
    					if(bots[j].curr_pos.position.x >= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y): # close bot is below and right
    						temp_x[i] = bots[i].curr_pos.position.x - 100*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    						#if (temp_x[i]<0):
    						#	temp_x[i] = 0
    						#elif (temp_x[i]>6):
    						#	temp_x[i] = 6	
    						temp_y[i] = bots[i].curr_pos.position.y - 100*(bots[j].curr_pos.position.x - bots[i].curr_pos.position.x)
    						#if (temp_y[i]<0):
    						#	temp_y[i] = 0
    						#elif (temp_y[i]>6):
    						#	temp_y[i] = 6	
    					
    					# CASE IV	
    					if(bots[j].curr_pos.position.x <= bots[i].curr_pos.position.x and bots[j].curr_pos.position.y <= bots[i].curr_pos.position.y): # close bot is below and left
    						temp_x[i] = bots[i].curr_pos.position.x - 100*(bots[i].curr_pos.position.y - bots[j].curr_pos.position.y)
    						#if (temp_x[i]<0):
    						#	temp_x[i] = 0
    						#elif (temp_x[i]>6):
    						#	temp_x[i] = 6	
    						temp_y[i] = bots[i].curr_pos.position.y + 100*(bots[i].curr_pos.position.x - bots[j].curr_pos.position.x)
    						#if (temp_y[i]<0):
    						#	temp_y[i] = 0
    						#elif (temp_y[i]>6):
    						#	temp_y[i] = 6
    					
    					# if there is a single j bot next to bot i, it needs to start changing instead of stopping the turn if the next bot is not close.
    					TooClose = True
    			
    			# this needs to be a separate if statement so the correct loop is broken out of. We did not know how to do a double break		
    			if (TooClose == True): # if a j robot is too close to the i robot, we're not even going to keep checking because the i robot needs to get turning ASAP
    				#print('TOO CLOSE')
    				break # breaks out of j iteration loop
    		
    		# below happens after j iteration loop is completed or broken out of but before the next i is iterated
    		# after all the j robots are checked for being too close, and none of them are, it is safe for the i robot to start going back to its original position
    		if (TooClose == False): 	# executes when the distance between the i robot and j robot (exculding itself) is no longer critical
    			# set dest position back to its original dest position
    			#print('SAFE')
    			temp_x[i] = x_dest[i]
    			temp_y[i] = y_dest[i]	
    			print(bots[i].name + 'bots[i].dest_pos.x')
    		
    	#	bots[i].setDestPosition(temp_x[i], temp_y[i])
    	#	bots[i].pub.publish(bots[i].dest_pos)
    		#TooClose==False already done above
    		
    		for i in range(0, len(robots)):
    			bots[i].setDestPosition(temp_x[i], temp_y[i])
    			bots[i].pub.publish(bots[i].dest_pos)
    		
    		
    		
    		
    		#coordList = build_hungarian(bots[i].curr_pos.position.x, bots[i].curr_pos.position.y, temp_x, temp_y)
    		#bots[i].setDestPosition(temp_x[i], temp_y[i])
    		
    		
    		#for bot in bots:
    		#	bot.setDestPosition(coordList[bot.name][0], coordList[bot.name][1])
    		#	bot.pub.publish(bot.dest_pos)
    	#	bots[i].setDestPosition(coordList[bots[i].name][0], coordList[bots[i].name][1])
    	#	#bots[i].setDestPosition(temp_x[i], temp_y[i])
    	#	bots[i].pub.publish(bots[i].dest_pos)
    			
    	# publish the update the new destination coordinates
    	
    	#coordList = build_hungarian(curr_x, curr_y, temp_x, temp_y)
    	
    	#for bot in bots:
    	#	bot.setDestPosition(coordList[bot.name][0], coordList[bot.name][1])
    	#	bot.pub.publish(bot.dest_pos)

bots = [] 
DEST_TOL = 0.1

if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # Assign number of robot masters

    for k in robots:
        bots.append(Master(k))
    
    # initializes temporary destination points to final destination points
    

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
    #for bot in bots:
    #    bot.setDestPosition(coordList[bot.name][0], coordList[bot.name][1])
    #    bot.pub.publish(bot.dest_pos)
        #print(bot.dest_pos)
        #the_x,the_y = bot.getCurrPos()
        #init_dist = ((x_dest[i] - the_x) ** 2 + (y_dest[i] - the_y) ** 2) ** 0.5 
        #print("running in main") 
    for i in range(0, len(robots)):
    	 x_dest[i] = coordList[bots[i].name][0]
    	 y_dest[i] = coordList[bots[i].name][1]
    # SHOULD I BE RUNNING THIS MULTIPLE TIMES?????????
    
    
    temp_x = x_dest
    temp_y = y_dest
    
    
    
    # At this point all initial setup is done. xrobot and yrobot are arrays of the initial positions. x_dest and y_dest are arrays of the desination points, not neccesarily ties to a specific robot.
    # curr_x, curr_y, temp_x, and temp_y are all still empty
    # curr_x and curr_y will be used from this point forward for current robot coorinates rather than the xrobot and yrobot.
    
    # Iterate through bots to get current positions and put the positions in the global arrays curr_x and curr_y. Then run airplane function for basic collision avoidance with updated current positions.
    while 1==1:
    
    	#Current

        # Current positions are bots[i].curr_pos.position.x and bots[i].curr_pos.position.y. We subscribe to this. Run airplane to check for collisions.
    	bot.airplane()
    	print("x0 error = ", bots[0].curr_pos.position.x-bots[0].dest_pos.x)
    	print("y0 error = ", bots[0].curr_pos.position.y-bots[0].dest_pos.y)
    	print("x1 error = ", bots[1].curr_pos.position.x-bots[1].dest_pos.x)
    	print("y1 error = ", bots[1].curr_pos.position.y-bots[1].dest_pos.y)
    	print("x2 error = ", bots[2].curr_pos.position.x-bots[2].dest_pos.x)
    	print("y2 error = ", bots[2].curr_pos.position.y-bots[2].dest_pos.y)
    	print("x3 error = ", bots[3].curr_pos.position.x-bots[3].dest_pos.x)
    	print("y3 error = ", bots[3].curr_pos.position.y-bots[3].dest_pos.y)
    	print("x4 error = ", bots[4].curr_pos.position.x-bots[4].dest_pos.x)
    	print("y4 error = ", bots[4].curr_pos.position.y-bots[4].dest_pos.y)
    	print("x5 error = ", bots[5].curr_pos.position.x-bots[5].dest_pos.x)
    	print("y5 error = ", bots[5].curr_pos.position.y-bots[5].dest_pos.y)
    	print("x6 error = ", bots[6].curr_pos.position.x-bots[6].dest_pos.x)
    	print("y6 error = ", bots[6].curr_pos.position.y-bots[6].dest_pos.y)
    	print("x7 error = ", bots[7].curr_pos.position.x-bots[7].dest_pos.x)
    	print("y7 error = ", bots[7].curr_pos.position.y-bots[7].dest_pos.y)
    	print("x8 error = ", bots[8].curr_pos.position.x-bots[8].dest_pos.x)
    	print("y8 error = ", bots[8].curr_pos.position.y-bots[8].dest_pos.y)
    	print("x9 error = ", bots[9].curr_pos.position.x-bots[9].dest_pos.x)
    	print("y9 error = ", bots[9].curr_pos.position.y-bots[9].dest_pos.y)
    	print("x10 error = ", bots[10].curr_pos.position.x-bots[10].dest_pos.x)
    	print("y10 error = ", bots[10].curr_pos.position.y-bots[10].dest_pos.y)
    	print("x11 error = ", bots[11].curr_pos.position.x-bots[11].dest_pos.x)
    	print("y11 error = ", bots[11].curr_pos.position.y-bots[11].dest_pos.y)
    	print("x12 error = ", bots[12].curr_pos.position.x-bots[12].dest_pos.x)
    	print("y12 error = ", bots[12].curr_pos.position.y-bots[12].dest_pos.y)
    	print("x13 error = ", bots[13].curr_pos.position.x-bots[13].dest_pos.x)
    	print("y13 error = ", bots[13].curr_pos.position.y-bots[13].dest_pos.y)
    	print("x14 error = ", bots[14].curr_pos.position.x-bots[14].dest_pos.x)
    	print("y14 error = ", bots[14].curr_pos.position.y-bots[14].dest_pos.y)
    	#print(bots[0].curr_pos.position.x)
    	#print(bots[0].curr_pos.position.y)
    	#print(bots[0].dest_pos.x)
    	#print(bots[0].dest_pos.y)
        #OLD
        
        # I don't really need to do this if I can get real time data in the airplane loop. This just introduces a lot of error. We subscribe to a constant stream of real time current data
    	#for i in range(0, len(x_robot)):
        #	x,y = bot.getCurrPos()
        #	xrobot[i]=x
        #	yrobot[i]=y
        #	bot.curr_pos.position.x = x
        #	bot.curr_pos.position.y = y
        
        	
        #k=0 # Needs to have a k because bots are not number iteratable? There's probobly a better way to do this...
    	#for bot in bots:
    	#	curr_x[k], curr_y[k] = bot.getCurrPos()
    	#	k = k+1
    	
    	# I don't like this because it uses bots[]
    	#for i in range(0, len(robots)):
    	#	curr_x[i], curr_y[i] = bots[i].getCurrPos()    	
    	
	#rospy.spin()
    