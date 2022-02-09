# Import important libraries
import roslib
import rospy
import time
import serial
import array
import numpy as np
import time
from scipy.optimize import linear_sum_assignment

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


while((c = getchar()) != EOF)
    print("%c\n", c)
#DICTIONARY FOR ALL LETTERS AND THEIR POSITIONS ON GRID
AX = [0.0, 0.6, 0.15, 0.45, 0.3, 0.3]
AY = [0.0, 0.0, 0.50, 0.50, 1.0, 0.5]

BX = [0.0, 0.0, 0.0, 0.60, 0.60, 0.40, 0.4, 0.4]
BY = [0.0, 0.5, 1.0, 0.25, 0.75, 0.50, 0.0, 1.0]

CX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6]
CY = [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 1.0]

DX = [0.0, 0.0, 0.0, 0.30, 0.30, 0.60]
DY = [0.0, 0.5, 1.0, 0.25, 0.75, 0.50]

EX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.6]
EY = [0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.0, 1.0]

FX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.6]
FY = [0.0, 0.5, 1.0, 0.5, 1.0, 1.0]

GX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6, 0.3]
GY = [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 1.0, 0.3, 0.3]

HX = [0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6]
HY = [0.0, 0.5, 1.0, 0.5, 0.0, 0.5, 1.0]

IX = [0.0, 0.3, 0.6, 0.3, 0.0, 0.3, 0.6]
IY = [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]

JX = [0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.0]
JY = [0.0, 0.3, 0.0, 0.5, 1.0, 1.0, 1.0]

KX = [0.0, 0.0, 0.0, 0.30, 0.30, 0.6, 0.6]
KY = [0.0, 0.5, 1.0, 0.25, 0.75, 0.0, 1.0]

LX = [0.0, 0.0, 0.0, 0.3, 0.6]
LY = [0.0, 0.5, 1.0, 0.0, 0.0]

MX = [0.0, 0.0, 0.0, 0.15, 0.3, 0.45, 0.6, 0.6, 0.6]
MY = [0.0, 0.5, 1.0, 0.75, 0.5, 0.75, 1.0, 0.5, 0.0]

NX = [0.0, 0.0, 0.0, 0.15, 0.3, 0.45, 0.6, 0.6, 0.6]
NY = [0.0, 0.5, 1.0, 0.75, 0.5, 0.25, 0.0, 0.5, 1.0]

OX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6]
OY = [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.5, 1.0]

PX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6]
PY = [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 1.0]

QX = [0.00, 0.0, 0.0, 0.20, 0.2, 0.4, 0.40, 0.4, 0.6]
QY = [0.25, 0.7, 1.0, 0.25, 1.0, 1.0, 0.25, 0.7, 1.0]

RX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6, 0.45]
RY = [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 1.0, 0.0, 0.25]

SX = [0.0, 0.0, 0.0, 0.00, 0.3, 0.3, 0.3, 0.6, 0.6, 0.6, 0.60]
SY = [0.0, 0.5, 1.0, 0.75, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.25]

TX = [0.0, 0.3, 0.3, 0.3, 0.6]
TY = [1.0, 0.0, 0.5, 1.0, 1.0]

UX = [0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6]
UY = [0.0, 0.5, 1.0, 0.0, 0.0, 0.5, 1.0]

VX = [0.0, 0.15, 0.3, 0.45, 0.6]
VY = [1.0, 0.50, 0.0, 0.50, 1.0]

WX = [0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6]
WY = [0.0, 0.5, 1.0, 0.0, 0.5, 0.0, 0.5, 1.0]

XX = [0.0, 0.0, 0.3, 0.6, 0.6]
XY = [0.0, 1.0, 0.5, 0.0, 1.0]

YX = [0.0, 0.3, 0.3, 0.6]
YY = [1.0, 0.5, 0.0, 0.5]

ZX = [0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.6]
ZY = [0.0, 1.0, 0.0, 0.5, 1.0, 0.0, 1.0]

#Robots
robots = ["usafabot0", "usafabot1", "usafabot2", "usafabot3", "usafabot4", "usafabot5", "usafabot6", "usafabot7", "usafabot8", "usafabot9", "usafabot10", "usafabot11", "usafabot12", "usafabot13", "usafabot14", "usafabot15", "usafabot16", "usafabot17", "usafabot18", "usafabot19", "usafabot20", "usafabot21", "usafabot22", "usafabot23", "usafabot24"]

rows = 25
cols = 25
hungarian = np.zeros((rows,cols))
assignments={}

def build_hungarian(x_robot, y_robot, x_dest, y_dest):
    for i in range(0, len(x_robot)): # find the distance for every starting point to each destination
        jobs = []
        for j in range(0, len(x_dest)):
            distance = ((x_dest[j] - x_robot[i]) ** 2 + (y_dest[j] - y_robot[i]) ** 2) ** 0.5
            distance = round(distance, 2)
            hungarian[i, j] = distance
    row_ind, col_ind = linear_sum_assignment(hungarian)
    #print(row_ind)    
    #print(col_ind)

    for robot, col in zip(robots, col_ind):
        assignments[robot]=[x_dest[col],y_dest[col]]
        
    return assignments


if __name__ == '__main__':
    start_time = time.time()
    build_hungarian()
