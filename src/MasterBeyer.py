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
# yrobot = [11, 12, 13, 14, 16, 17, 18, 19, 30, 30, 30, 30, 30, 30, 30, 17, 16, 14, 13, 0, 0, 0, 0, 0, 0]
# xrobot = [0, 0, 0, 0, 0, 0, 0, 0, 12, 13, 14, 16, 17, 18, 19, 30, 30, 30, 30, 18, 17, 16, 14, 13, 12]
robotDestination = []

# TODO: change this list to match number of robots
# must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']

# x_dest = [10, 9, 8, 8, 10, 9, 8, 8, 12, 12, 13, 16, 16, 17, 17, 21, 20, 20, 21, 16, 17, 16, 13, 12, 12]
# y_dest = [14, 13, 13, 14, 15, 16, 16, 15, 15, 16, 16, 15, 16, 15, 16,16, 15, 14, 13, 14, 13, 13, 14, 14, 13]
# x_dest = [0, .5,  1, 0, 1, 0, .5, 0, 1.75, 1.75, 2.5, 3, 3, 3.5, 4, 5.25, 4.5, 4.5, 5.25, 3, 4, 3, 2.25, 1.75, 1.75]

# y_dest = [1, 1.5, 2, 2, 3, 3, 3.5, 4, 3, 4, 4, 3, 4, 3, 4, 4, 3, 2, 1, 2, 1, 1, 3, 2, 1]

x_dest = [3, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4, 3.5, 3, 2.5]

y_dest = [2, 5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4, 4, 4, 4, 4, 4, 3.5, 3, 2.5, 2, 1.5, 1, 1, 1, 1, 1]


# Define the Controller class
class Master:

    def __init__(self, TIBot):
        self.gnd_curr = Pose()
        self.gnd_dest = Point()
        self.TIBot = TIBot

        # -----------------------------------------------------------------------------
        # Topics and Timers
        # -----------------------------------------------------------------------------
        # Publish to the controller
        self.pub = rospy.Publisher(TIBot + '/ti_dest_pos', Point, queue_size=10)
        rospy.Timer(rospy.Duration(.1), self.callback_groundTalker)

        # Listen for the bots' current position to the controller
        rospy.Subscriber(TIBot + '/ti_curr_pos', Pose, self.callback_groundListener)
        # rospy.Subscriber(TIBot + '/odom', Odometry, self.callback_groundListener)

    # -------------------------------------------------------------------------------
    # Class Functions
    # -------------------------------------------------------------------------------
    def setGroundDestPosition(self, x, y):
        # Data based on drone position
        self.gnd_dest.x = x
        self.gnd_dest.y = y

    def callback_groundTalker(self, data):
        # message is printed to the terminal, written to Node log file, and written to rosout
        # rospy.loginfo(self.gnd_dest)
        self.pub.publish(self.gnd_dest)  # work being done
        # rate.sleep()

    def callback_groundListener(self, data):
        self.gnd_curr.position.x = data.position.x
        self.gnd_curr.position.y = data.position.y
        self.gnd_curr.orientation.z = data.orientation.z
        # rospy.loginfo(rospy.get_caller_id() + 'I heard position:%s' %(self.gnd_curr))

    def getCurrentPos(self):
        return self.gnd_curr.position.x, self.gnd_curr.position.y;


bots = []


def run():
    # Code can be added in this function to add functionality to particular master threads
    pass


# an alphabet-list of letter-arrays holding coordinate-arrays idea for later: add third coordinate to simulate weight
# of position so that only the important positions must be filled when low on number of robots this is built for a
# coordinate system of 4 by 4 inch squares or 10 x 10 cm square units, letters are 7x8units
def alphabet(letter):
    A = [[1, 1], [1, 2], [1, 3], [1, 4], [2, 2], [2, 4], [3, 1], [3, 2], [3, 3], [3, 4]]
    B = [[1, 1], [1, 2], [1, 3], [1, 4], [2, 1], [2, 2]]
    C = [[1, 2], [1, 3], [2, 1], [2, 4]]
    D = [[1, 1], [1, 2], [1, 3], [1, 4], [2, 1], [2, 4], [3, 2], [3, 3]]
    E = [[1, 1], [1, 2], [1, 3], [1, 4], [2, 1], [2, 3], [2, 4]]
    F = [[1, 1], [1, 2], [1, 3], [1, 4], [2, 2], [2, 4]]
    alphabet_list = [A, B, C, D, E, F]
    if letter == 'A':
        return A
    elif letter == 'B':
        return B
    elif letter == 'C':
        return C
    elif letter == 'D':
        return D
    elif letter == 'E':
        return E
    elif letter == 'F':
        return F
    else:
        return alphabet_list


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

    # Take input from the user
    # word = input("Enter word (uppercase only, 7 letters or less, only A-F for now): ")
    word = "DFEC"
    # numRobots = int(input("Enter number of robots available (up to 32 robots. must match robots list length): "))

    # Check if number entered by user matches robot list length
    # if numRobots != robots.length:
    #    print("robot list length does not match number of robots entered")

    # Assign number of robot masters
    for k in robots:
        bots.append(Master(k))

    # Get initial bot positions
    for bot in bots:
        x = 0
        y = 0
        # block until bot's RR is operational
        while x == 0 and y == 0:
            x, y = bot.getCurrentPos()
        xrobot.append(x)
        yrobot.append(y)
        print("Completed bot: " + bot.TIBot)

    # Calculation functions
    coordList = wordToPoints(word)

    # robotPlacement(numRobots)

    # robotAssignment(numRobots)

    print("x = ", listX)
    print("y = ", listY)
    print("Comined coordList = ", coordList)

    print("Robot Init X = ", xrobot)
    print("Robot Init Y = ", yrobot)
    print("Combined RobotDestination = ", robotDestination)

    # Assign final bot destinations
    i = 0

    for bot in bots:
        bot.setGroundDestPosition(x_dest[i], y_dest[i])
        curr_x, curr_y = bot.getCurrentPos()
        init_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5

        while True:
            curr_x, curr_y = bot.getCurrentPos()
            curr_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5
            if (init_dist - curr_dist) > .25:
                i += 1
                print(bot.TIBot + " is complete")
                bot.setGroundDestPosition(0, 0)
                break

    rospy.spin()
