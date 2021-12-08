#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground and Air Robot Teaming Capstone: Ground Controller
Date: 29 Nov 2021
----------------------------------------------------------------------------------"""

# Import important libraries
import roslib
import rospy
import time
import serial
import array
import signal
import csv
from std_msgs.msg import String
from threading import Thread

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from datetime import datetime
from statistics import mean

# TODO: what if you have more or less robots than needed?

# Global Variables
xrobot = []
yrobot = []
robotDestination = []
DEST_DIST = .25  # meters
TIMEOUT_THRESH = 10  # seconds
# Kill state magic number
KILL_SIG = 22
# Number of points to use for pos avg err
NUM_AVG_CYCLES = 5
# Time interval in second for pos avg
AVG_WAIT_TIME = 10
# must match number of robots entered by user
robots = ['usafabot0', 'usafabot1', 'usafabot2', 'usafabot3', 'usafabot4',
          'usafabot5', 'usafabot6', 'usafabot7', 'usafabot8', 'usafabot9',
          'usafabot10', 'usafabot11', 'usafabot12', 'usafabot13', 'usafabot14',
          'usafabot15', 'usafabot16', 'usafabot17', 'usafabot18', 'usafabot19',
          'usafabot20', 'usafabot21', 'usafabot22', 'usafabot23', 'usafabot24']
# DFEC from inside to outside
x_dest = [2.3, 2, 1.6, 1.3, 1.3, 1, 1, 1, 2, 2.3, 2.6, 3, 3.3, 3.6, 4, 4, 4, 4.6, 4.6, 3.6, 3.3, 3.3, 3, 3, 2]
y_dest = [2.5, 2.5, 2.5, 2.75, 2.25, 3, 2.5, 2, 3, 3, 3, 3, 3, 3, 3, 2, 2.5, 2, 3, 2, 2.5, 2, 2.5, 2, 2]

# CSV filename for error measurement
filename = datetime.now().strftime("%d%b%Y-%H:%M:%S") + ".csv"
# CSV Headers
fields = ['bot', 'x_avg_pos', 'y_avg_pos', 'pos_err', 'time']


def handler(signum, frame):
    stop_bots()
    print("KILLED with CTRL_C")
    exit(1)


def measure_error(num_samples, sample_period):
    print("Collecting ", num_samples, " taken every ", sample_period, " seconds")
    for Cycle in range(1, NUM_AVG_CYCLES):
        for robot in bots:
            # Add sample to array
            robot.x_avg.append(robot.curr_pos.position.x)
            robot.y_avg.append(robot.curr_pos.position.y)
        # Wait specified number of seconds before taking another sample
        time.sleep(sample_period)
        print("Finished cycle: ", Cycle)

    with open(filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(fields)
        for robot in bots:
            robot.pos_err = ((mean(robot.x_avg) - robot.dest_pos.x) ** 2 + (
                    mean(robot.y_avg) - robot.dest_pos.y) ** 2) ** .5
            row = robot.name + str(mean(robot.x_avg)) + str(mean(robot.y_avg)) + str(robot.pos_err) + str(robot.time)
            print(row)
            csvwriter.writerow([row])
    print("CSV created with filename: ", filename)


def stop_bots():
    for robot in bots:
        robot.setGroundDestPosition(KILL_SIG, KILL_SIG)
        robot.pub.publish(bot.dest_pos)
        print("Stopping bot: ", robot.name)


# Define the Controller class
class Master:

    def __init__(self, USAFABOT):
        self.curr_pos = Pose()
        self.dest_pos = Point()
        self.name = USAFABOT
        self.pos_err = 0
        self.x_avg = []
        self.y_avg = []
        self.timeout = False
        self.time = 0
        # -----------------------------------------------------------------------------
        # Topics and Timers
        # -----------------------------------------------------------------------------
        # Publish to the controller
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size=10)
        # rospy.Timer(rospy.Duration(.1), self.callback_Pos)

        # Listen for the bots' current position to the controller
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)

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

    def getCurrPos(self):
        return self.curr_pos.position.x, self.curr_pos.position.y

    def setDestPosition(self, x, y):
        self.dest_pos.x = x
        self.dest_pos.y = y


bots = []
signal.signal(signal.SIGINT, handler)
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
        tic = time.perf_counter()
        while x == 0 and y == 0:
            x, y = bot.getCurrPos()
            if time.perf_counter() - tic > TIMEOUT_THRESH:
                bot.timeout = True
                print("Timeout: " + bot.name)
                break
            elif x != 0 or y != 0:
                xrobot.append(x)
                yrobot.append(y)
                toc = time.perf_counter()
                t = toc - tic
                print("Completed bot: " + bot.name)
                print(t)

    print(xrobot)
    print(yrobot)

    #    print("Combined RobotDestination = ", x_dest, y_dest)

    # Assign final bot destinations
    i = 0

    for bot in bots:
        if not bot.timeout:
            bot.setGroundDestPosition(x_dest[i], y_dest[i])
            bot.pub.publish(bot.dest_pos)
            print("Dest set for:" + bot.name)
            tic = time.perf_counter()
            curr_x, curr_y = bot.getCurrPos()
            # init_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5
            while True:
                bot.setGroundDestPosition(x_dest[i], y_dest[i])
                bot.pub.publish(bot.dest_pos)
                curr_x, curr_y = bot.getCurrPos()
                bot.setGroundDestPosition(x_dest[i], y_dest[i])
                bot.pub.publish(bot.dest_pos)
                # print(curr_x, curr_y)
                curr_dist = ((x_dest[i] - curr_x) ** 2 + (y_dest[i] - curr_y) ** 2) ** 0.5
                if curr_dist < DEST_DIST:
                    toc = time.perf_counter()
                    bot.time = round(toc - tic, 4)
                    print(bot.name + " is complete. It took " + str(bot.time) + " seconds")
                    bot.setGroundDestPosition(0, 0)  # need this?
                    break
        else:
            print("Skipping bot", bot.name)
        i+=1

    print("all bots complete")
    stop_bots()
    measure_error(NUM_AVG_CYCLES, AVG_WAIT_TIME)
    rospy.spin()
