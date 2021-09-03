#!/usr/bin/env python3

# Import important libraries
import rospy
import time
from hungarian import build_hungarian

from geometry_msgs.msg import Point, Pose

# TODO: take current robot position instead of assigning initial positions - fixed
# TODO: what if you have more or less robots than needed? 


# TODO: change this list to match number of robots
#must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']

#Destination Points
x_dest = [1.0, 1.0, 1.0, 1.30, 1.30, 1.60, 2.0, 2.0, 2.0, 2.3, 2.3, 2.6, 3.0, 3.0, 3.0, 3.3, 3.3, 3.3, 3.6, 3.6, 4.0, 4.0, 4.0, 4.6, 4.6]
y_dest = [3.0, 2.5, 2.0, 2.82, 2.18, 2.5, 3.0, 2.5, 2.0, 3.0, 2.5, 3.0, 3.0, 2.5, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0]

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
        # Data based on drone position
        self.dest_pos.x = x
        self.dest_pos.y = y


bots = [] #not sure what this guy does


DEST_TOL = 0.1 #also not sure what this guy does

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
        while(x == 0 and y == 0):
            x,y = bot.getCurrPos()
        xrobot.append(x)
        yrobot.append(y)
        print("Completed bot: " + bot.name)

    # Calculation functions
    print(xrobot)
    print(yrobot)
    coordList = build_hungarian(xrobot, yrobot, x_dest, y_dest)
    
    print("Comined coordList = ", coordList)

    print("Robot Init X = ", xrobot)
    print("Robot Init Y = ", yrobot)

    # Assign final bot destinations
    i = 0

    for bot in bots:
        bot.setDestPosition(coordList[bot.name][0], coordList[bot.name][1])
        bot.pub.publish(bot.dest_pos)
        print(bot.dest_pos)
        curr_x,curr_y = bot.getCurrPos()
        init_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5        
    

    rospy.spin()












