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

	destmult = 10000
	potPointX = []
	potPointY = []
	potBotX = []
	potBotY = []
	vectorX = []
	vectorY = []
	sumX = []
	sumY = []
	vector = (vectorX, vectorY)
	
	for bot in range(0, len(xrobot)):

		pointDistX = coordList[robots[bot]][0]-xrobot[bot]
		pointDistY = coordList[robots[bot]][1]-yrobot[bot]

		# Get potential for x between bot and final destination
		if(pointDistX > 0.05):
			potPointX.append(round(1/(pointDistX ** 2), 3))
		elif (pointDistX <= 0.05 and pointDistX >=-0.05):
			potPointX.append(0)
		else:
			potPointX.append(-1*round(1/(pointDistX ** 2), 3))
		
		# Get potential for y between bot and final destination
		if(pointDistY > 0.05):
			potPointY.append(round(1/(pointDistY ** 2), 3))
		elif (pointDistY <= 0.05 and pointDistY >=-0.05):
			potPointY.append(0)
		else:
			potPointY.append(-1*round(1/(pointDistY ** 2), 3))
			
		# TODO: only check repulsion of N robots within XY distance
		for i in range(0, len(xrobot)):
			distanceX = round(xrobot[i] - xrobot[bot], 3)
			distanceY = round(yrobot[i] - yrobot[bot], 3)
		#CHECK WITHIN N METER
			if(abs(distanceX) <= 0.5):
				if (distanceX != 0):
					if (distanceX > 0):
						potBotX.append(-1*(round((1/(distanceX**2)), 3)))
					elif (distanceX < 0):
						potBotX.append((round((1/(distanceX**2)), 3)))
			else:
				potBotX.append(0)
			if(abs(distanceY) <= 0):		
				if (distanceY != 0):
					if (distanceY > 0):
						potBotY.append(-1*(round((1/(distanceY**2)), 3)))
					elif (distanceY < 0):
						potBotY.append((round((1/(distanceY**2)), 3)))
			else:
				potBotY.append(0)
					
		sumX.append(sum(potBotX) + destmult*potPointX[bot])
		sumY.append(sum(potBotY) + destmult*potPointY[bot])
		
	for bot in range(0, len(xrobot)):
		
		if (sumX[bot] > 0):
			if((sumX[bot]/max(sumX)) < 0.1):
				vectorX.append(0.1)
			else:
				vectorX.append(sumX[bot]/max(sumX))
		elif (sumX[bot] < 0):
			if(abs((sumX[bot]/min(sumX))) < 0.1):
				vectorX.append(-0.1)
			else:
				vectorX.append(-abs(sumX[bot]/min(sumX)))
				
		else:
			vectorX.append(0)
				
		if (sumY[bot] > 0):
			if((sumY[bot]/max(sumY)) < 0.1):
				vectorY.append(0.1)
			else:
				vectorY.append(sumY[bot]/max(sumY))
		elif (sumY[bot] < 0):
			if(abs(sumY[bot]/min(sumY)) < 0.1):
				vectorY.append(-0.1)
			else:
				vectorY.append(-abs(sumY[bot]/min(sumY)))
		else:
			vectorY.append(0)
			

	return np.vstack((vectorX, vectorY)).T
		
		
