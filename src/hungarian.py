
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
robots = ["TIBOT0", "TIBOT1", "TIBOT2", "TIBOT3", "TIBOT4", "TIBOT5", "TIBOT6", "TIBOT7", "TIBOT8", "TIBOT9", "TIBOT10", "TIBOT11", "TIBOT12", "TIBOT13", "TIBOT14", "TIBOT15", "TIBOT16", "TIBOT17", "TIBOT18", "TIBOT19", "TIBOT20", "TIBOT21", "TIBOT22", "TIBOT23", "TIBOT24"]



#Starting Points
x_robot = [0, 0, 0, 0, 0, 0, 0.75, 1.50, 2.25, 3.0, 3.75, 4.25, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.25, 0.75, 1.50, 2.25, 3.0, 3.75, 4.25]
y_robot = [0.75, 1.50, 2.25, 3.0, 3.75, 4.25, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.75, 1.50, 2.25, 3.0, 3.75, 4.25, 0, 0, 0, 0, 0, 0, 0]

#Destination Points
x_dest = [1.75, 1.50, 1.25, 1.25, 1.75, 1.50, 1.25, 1.25, 2.25, 2.25, 2.50, 3.25, 3.00, 3.00, 3.25, 3.75, 4.00, 3.75, 4.00, 3.00, 3.25, 3.00, 2.50, 2.25, 2.25]
y_dest = [2.25, 1.75, 1.75, 2.25, 2.75, 3.25, 2.75, 3.25, 2.75, 3.25, 3.25, 2.75, 2.75, 3.25, 3.25, 2.75, 3.25, 2.25, 1.75, 2.25, 1.75, 1.75, 2.75, 2.25, 1.75]

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
