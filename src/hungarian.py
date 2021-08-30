
# Import important libraries
import roslib
import rospy
import time
import serial
import array
import numpy as np
import time
from scipy.optimize import linear_sum_assignment

#Robots
robots = ["usafabot0", "usafabot1", "usafabot2", "usafabot3", "usafabot4", "usafabot5", "usafabot6", "usafabot7", "usafabot8", "usafabot9", "usafabot10", "usafabot11", "usafabot12", "usafabot13", "usafabot14", "usafabot15", "usafabot16", "usafabot17", "usafabot18", "usafabot19", "usafabot20", "usafabot21", "usafabot22", "usafabot23", "usafabot24"]



#Starting Points
x_robot = [0, 0, 0, 0, 0, 0, 0.75, 1.50, 2.25, 3.0, 3.75, 4.25, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.25, 0.75, 1.50, 2.25, 3.0, 3.75, 4.25]
y_robot = [0.75, 1.50, 2.25, 3.0, 3.75, 4.25, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.75, 1.50, 2.25, 3.0, 3.75, 4.25, 0, 0, 0, 0, 0, 0, 0]

#Destination Points
x_dest = [1.0, 1.0, 1.0, 1.30, 1.30, 1.60, 2.0, 2.0, 2.0, 2.3, 2.3, 2.6, 3.0, 3.0, 3.0, 3.3, 3.3, 3.3, 3.6, 3.6, 4.0, 4.0, 4.0, 4.6, 4.6]
y_dest = [3.0, 2.5, 2.0, 2.75, 2.25, 2.5, 3.0, 2.5, 2.0, 3.0, 2.5, 3.0, 3.0, 2.5, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0]

rows = 25
cols = 25
hungarian = np.zeros((rows,cols))
assignments={}




def build_hungarian():
    for i in range(0, len(x_robot)): # find the distance for every starting point to each destination
        jobs = []
        for j in range(0, len(x_dest)):
            distance = ((x_dest[j] - x_robot[i]) ** 2 + (y_dest[j] - y_robot[i]) ** 2) ** 0.5
            distance = round(distance, 2)
            hungarian[i, j] = distance
    print("hungarian: ", hungarian)
    row_ind, col_ind = linear_sum_assignment(hungarian)
    print(row_ind)    
    print(col_ind)

    for robot, col in zip(robots, col_ind):
        assignments[robot]=[x_dest[col],y_dest[col]]
        
    print(assignments)


if __name__ == '__main__':
    start_time = time.time()
    build_hungarian()
    print("Program took: ", time.time() - start_time)
