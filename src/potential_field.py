#!/usr/bin/env python3

# Import important libraries
import rospy
import math

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
	vector = {}
	
	for bot in range(0, len(xrobot)):
		pointDistX.append(abs(coordList[robots[bot]][0]-xrobot[bot]))
		pointDistY.append(abs(coordList[robots[bot]][1]-yrobot[bot]))
		
		for i in range(0, len(xrobot)):
				distanceX = xrobot[i] - xrobot[bot]
				distanceY = yrobot[i] - yrobot[bot]
				botDistX.append(abs(distanceX)) 
				botDistY.append(abs(distanceY))
	
	for bot in range(0, len(xrobot)):
		sumX = 0
		sumY = 0
		
		for i in range(0, len(xrobot)):
			sumX = sumX - botDistX[i]
			sumY = sumY - botDistY[i]
		
		sumX = sumX + destmult*pointDistX[bot]
		sumY = sumY + destmult*pointDistY[bot]
		
		for robot in zip(robots):
		
			vector[robot] = [sumX, sumY]

	return vector
		
		
