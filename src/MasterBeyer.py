#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground and Air Robot Teaming Capstone: Ground Controller
Date: 10 Feb 2020
----------------------------------------------------------------------------------"""

# Import important libraries
import roslib
import rospy
import time
import serial
import array
from std_msgs.msg import String
from threading import Thread

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# TODO: take current robot position instead of assigning initial positions - fixed
# TODO: what if you have more or less robots than needed?

# Global Variables
listX = []
listY = []
xrobot = []
yrobot = []
robotDestination = []

# TODO: change this list to match number of robots
# must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']
# This spells out DFEC, but causes collisions TODO: Redo order to work inside out
x_dest = [1.0, 1.0, 1.0, 1.30, 1.30, 1.60, 2.0, 2.0, 2.0, 2.3, 2.3, 2.6, 3.0, 3.0, 3.0, 3.3, 3.3, 3.3, 3.6, 3.6, 4.0, 4.0, 4.0, 4.6, 4.6]
y_dest = [3.0, 2.5, 2.0, 2.75, 2.25, 2.5, 3.0, 2.5, 2.0, 3.0, 2.5, 3.0, 3.0, 2.5, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0, 3.0, 2.5, 2.0, 3.0, 2.0]

x_dest = [2.3,2,1.6,1.3,1.3,1,1,1,2,2.3,2.6,3,3.3,3.6,4,4,4,4.6,4.6,3.6,3.3,3.3,3,3,2]
y_dest = [2.5,2.5,2.5,2.75,2.25,3,2.5,2,3,3,3,3,3,3,3,2,2.5,2,3,2,2.5,2,2.5,2,2]

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
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size=10)
        #rospy.Timer(rospy.Duration(.1), self.callback_Pos)

        # Listen for the bots' current position to the controller
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)
        # rospy.Subscriber(TIBot + '/odom', Odometry, self.callback_groundListener)

    # -------------------------------------------------------------------------------
    # Class Functions
    # -------------------------------------------------------------------------------
    def setGroundDestPosition(self, x, y):
        # Data based on drone position
        self.dest_pos.x = x
        self.dest_pos.y = y

    def callback_currPos(self, data):
        self.curr_pos.position.x = round(data.position.x, 3)
        self.curr_pos.position.y = round(data.position.y, 3)
        self.curr_pos.orientation.z = round(data.orientation.z, 3)
        
    def callback_groundListener(self, data):
        self.curr_pos.position.x = data.position.x
        self.curr_pos.position.y = data.position.y
        self.curr_pos.orientation.z = data.orientation.z
        # rospy.loginfo(rospy.get_caller_id() + 'I heard position:%s' %(self.gnd_curr))

        
    def getCurrPos(self):
        return self.curr_pos.position.x, self.curr_pos.position.y;

    def setDestPosition(self, x, y):
        self.dest_pos.x = x
        self.dest_pos.y = y 


bots = []




# converts word to Grid coordinates
def wordToPoints(word):
    coordList = []
    tempLetter = []
    tempWord = []
    tempWordShifted = []
    global listX
    global listY

    for m in range(0, len(word)):  # fill tempWord with unshifted coordinate arrays
        tempLetter = alphabet(word[m])
        tempWord.append(tempLetter)

    tempWordShifted = shiftLetters(tempWord, len(word))
    for n in range(0, len(tempWordShifted)):
        for p in range(0, len(tempWordShifted[n])):
            coordList.append(tempWordShifted[n][p])
            listX.append(tempWordShifted[n][p][0])
            listY.append(tempWordShifted[n][p][1])
    return coordList


# shifts letter coordinates to not be centered at 0,0 and fit our frame
def shiftLetters(word, size):
    mid = int(size / 2)  # to the right of true middle
    for i in range(mid, len(word)):
        for j in range(0, len(word[i])):
            word[i][j][0] += 4 * (i - mid) + 15
            word[i][j][1] += 12
    for h in range(0, mid):
        for k in range(0, len(word[h])):
            word[h][k][0] -= 4 * (mid - h)
            word[h][k][0] += 15
            word[h][k][1] += 12
    return word


# fill xrobot and yrobot with initial robot coordinates
def robotPlacement(numRobots):
    count = 0
    global xrobot
    global yrobot
    for i in range(1, numRobots + 1):
        if i <= 4:  # only works for up to 32 robots
            count = 0
        elif i <= 8:
            count = 1
        elif i <= 12:
            count = -1
        elif i <= 16:
            count = 2
        elif i <= 20:
            count = -2
        elif i <= 24:
            count = 3
        elif i <= 28:
            count = -3
        elif i <= 32:
            count = 4

        if i % 4 == 1:  # 1,5,9,13,17,21,25
            xrobot.append(0)
            yrobot.append(15 + count)
        elif i % 4 == 2:  # 2,6,10,14,18,22
            xrobot.append(15 + count)
            yrobot.append(30)
        elif i % 4 == 3:  # 3,7,11,15,19,23
            xrobot.append(30)
            yrobot.append(15 + count)
        elif i % 4 == 0:  # 4,8,12,16,20,24
            xrobot.append(15 + count)
            yrobot.append(0)


# match robot coordinates with letter coordinates
def robotAssignment(numRobots):
    global robotDestination
    letterx = listX.copy()
    lettery = listY.copy()
    robotDestination = [None] * numRobots  # move this line down two lines to not fill any unsused robots with "none"
    if numRobots > len(listX):
        numRobots = len(listX)

    for i in range(0, numRobots - 1):  # iterate each robot
        maxDistance = 0
        pos = 0
        for j in range(0, len(letterx)):  # iterate each letter
            distance = ((xrobot[i] - letterx[j]) ** 2 + (yrobot[i] - lettery[j]) ** 2) ** 0.5
            if (xrobot[i] < 5 and letterx[j] < 11):  # letter D
                if ((yrobot[i] >= 15 and lettery[j] >= 15) or  # top half
                        (yrobot[i] < 15 and lettery[j] < 15)):  # bottom half
                    if distance > maxDistance:
                        maxDistance = distance
                        robotDestination[i] = [letterx[j] / 5, lettery[j] / 5]
                        pos = j

            elif (1 < xrobot[i] <= 15 and 11 < letterx[j] < 14):  # letter F
                if ((yrobot[i] > 20 and lettery[j] >= 15) or  # top half
                        (yrobot[i] < 5 and lettery[j] < 15)):  # bottom half
                    if distance > maxDistance:
                        maxDistance = distance
                        robotDestination[i] = [letterx[j] / 5, lettery[j] / 5]
                        pos = j
            elif (15 < xrobot[i] <= 27 and 15 < letterx[j] <= 19):  # letter E
                if ((yrobot[i] > 20 and lettery[j] >= 15) or  # top half
                        (yrobot[i] < 5 and lettery[j] < 15)):  # bottom half
                    if distance > maxDistance:
                        maxDistance = distance
                        robotDestination[i] = [letterx[j] / 5, lettery[j] / 5]
                        pos = j
            elif (xrobot[i] > 25 and letterx[j] > 19):  # letter C
                if ((yrobot[i] >= 15 and lettery[j] >= 15) or  # top half
                        (yrobot[i] < 15 and lettery[j] < 15)):  # bottom half
                    if distance > maxDistance:
                        maxDistance = distance
                        robotDestination[i] = [letterx[j] / 5, lettery[j] / 5]
                        pos = j
        del letterx[pos]
        del lettery[pos]
    robotDestination[numRobots - 1] = [letterx[0] / 5, lettery[0] / 5]


DEST_TOL = 0.1

if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # Assign number of robot masters
    for k in robots:
        bots.append(Master(k))

#    Get initial bot positions
    for bot in bots:
        x = 0
        y = 0
        # block until bot's RR is operational
        print("Waiting bot: " + bot.name)
        while x == 0 and y == 0:
           x, y = bot.getCurrPos()
        xrobot.append(x)
        yrobot.append(y)
        print("Completed bot: " + bot.name)

    # Calculation functions
 #   coordList = wordToPoints(word)
    print(xrobot)
    print(yrobot)


    print("x = ", listX)
    print("y = ", listY)
#    print("Comined coordList = ", coordList)

    print("Robot Init X = ", xrobot)
    print("Robot Init Y = ", yrobot)
    print("Combined RobotDestination = ", x_dest, y_dest)

    # Assign final bot destinations
    i = 0

    for bot in bots:
        bot.setGroundDestPosition(x_dest[i], y_dest[i])
        print("Dest set for:" + bot.name)
        bot.pub.publish(bot.dest_pos)
        curr_x, curr_y = bot.getCurrPos()
        init_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5

        while True:
            curr_x, curr_y = bot.getCurrPos()
            #print(curr_x, curr_y)
            curr_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5
            if curr_dist < .25 :
                i += 1
                print(bot.name + " is complete")
                bot.setGroundDestPosition(0, 0)
                break
                
                
    print("all bots complete")
    rospy.spin()
