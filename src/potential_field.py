#!/usr/bin/env python3

# Import important libraries
import rospy
import math
import numpy as np

from geometry_msgs.msg import Point, Pose

robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']
          
def potential_field(xrobot, yrobot, coordList):

	destmult = 25 # Multiplier to pull robot to destination stronger than repulsion from other bots
	potPointX = [] # Vector of rounded distances between each robot and its destination
	potPointY = []
	potBotX = [] # Vector of rounded distances between each robot to eachother
	potBotY = []
	vectorX = [] # Vector of each robots total vector i.e combination of destination and obstacles 
	vectorY = []
	vector = (vectorX, vectorY) # Is this used??
	
	for bot in range(0, len(xrobot)):
		# Find current distance componentwise between robot and destination
		pointDistX = coordList[robots[bot]][0]-xrobot[bot] 
		pointDistY = coordList[robots[bot]][1]-yrobot[bot]
		
		
		# Append each individual distance to the array
		if(pointDistX > 0):
			potPointX.append(round(1/(pointDistX ** 2), 3))
		else:
			potPointX.append(-1*round(1/(pointDistX ** 2), 3))
			
		if(pointDistY > 0):
			potPointY.append(round(1/(pointDistY ** 2), 3))
		else:
			potPointY.append(-1*round(1/(pointDistY ** 2), 3))
			
		# Find the distance of each robot to each other
		
		for i in range(0, len(xrobot)):
				distanceX = round(xrobot[i] - xrobot[bot], 3)
				distanceY = round(yrobot[i] - yrobot[bot], 3)
				# Check that robot is not repelling itself
				if (distanceX != 0):
					if (distanceX > 0):
						potBotX.append(-1*(round((1/(distanceX**2)), 3)))
					elif (distanceX < 0):
						potBotX.append((round((1/(distanceX**2)), 3)))
				else:
					potBotX.append(0)
						
				if (distanceY != 0):
					if (distanceY > 0):
						potBotY.append(-1*(round((1/(distanceY**2)), 3)))
					elif (distanceX < 0):
						potBotY.append((round((1/(distanceY**2)), 3)))
				else:
					potBotY.append(0)	
		# 
		vectorX.append(sum(potBotX) + destmult*potPointX[bot])
		vectorY.append(sum(potBotY) + destmult*potPointY[bot])
		
			
	return np.vstack((vectorX, vectorY)).T
		
		
