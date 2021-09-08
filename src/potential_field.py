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

	destmult = 25
	pointDistX = []
	pointDistY = []
	botDistX = []
	botDistY = []
	vectorX = []
	vectorY = []
	vector = (vectorX, vectorY)
	
	for bot in range(0, len(xrobot)):
		sumX = 0
		sumY = 0
		
		pointDistX.append(round(abs(1/((coordList[robots[bot]][0]-xrobot[bot])**2)), 3))
		pointDistY.append(round(abs(1/((coordList[robots[bot]][1]-yrobot[bot])**2)), 3))
		
		for i in range(0, len(xrobot)):
				distanceX = xrobot[i] - xrobot[bot]
				distanceY = yrobot[i] - yrobot[bot]

				if (distanceX != 0 and distanceY != 0):
					botDistX.append(round(abs(1/(distanceX**2)), 3))
					botDistY.append(round(abs(1/(distanceY**2)), 3))	
				
				else:
					botDistX.append(0)
					botDistY.append(0)	
		
		for i in range(0, len(xrobot)):
			sumX = sumX - botDistX[i]
			sumY = sumY - botDistY[i]
		
		sumX = sumX + destmult*pointDistX[bot]
		sumY = sumY + destmult*pointDistY[bot]
		

		vectorX.append(sumX)
		vectorY.append(sumY)
			
	return np.vstack((vectorX, vectorY)).T
		
		
