#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose, Point, Twist

from squaternion import Quaternion

import tf2_ros
import tf2_msgs.msg

## Controller
#
#  Subscribes to TI_Bot's current position (from Roadrunner) and destination 
#   position (from master node) to determine linear and angular velocity and
#   publishes those values to the TI_Bot
#
#  Subscriber
#   Topic: ti_curr_pos
#     Msg type: Pose
#     Freq: 100 Hz
#   Topic: ti_dest_pos
#     Msg type: Point
#     Freq: 100 Hz
#
#  Publisher
#   Topic: cmd_vel
#     Msg type: Twist
#     Freq: 100 Hz
class Controller:
    # heading tolerance +/- degrees
    HDG_TOL = 25
    # destination tolerance +/- meters
    DEST_TOL = 0.05
    # roation controller constant
    K_HDG = 0.01

    def __init__(self):
        # instance variables unique to each instance of class
        self.robot = Pose()
        self.target = Pose()
        self.twist = Twist()
        self.running = False

        # Frequency of publisher - 100 Hz
        self.rate = rospy.Rate(100)

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        rospy.Subscriber('/target_avg', Pose, self.callback_target)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Timer(rospy.Duration(.01), self.callback_converter)
    
    def callback_target(self, data):
        self.target = data

    def headingConvert(self, yaw):
        if yaw < 0:
            return 360 + yaw
        else:
            return yaw

    # Callback function that calculates orientation offset needed to navigate
    # the TI_Bot to the destination. Provides a linear/angular velocity
    # to TI_Bot using a proportional controller based on the orientation offset
    # Frequency: 100 Hz
    def callback_converter(self, event):  
        try:
            #trans = self.tfBuffer.lookup_transform('base_link', 'tag_0', rospy.Time(0))
            bot = self.tfBuffer.lookup_transform('world', 'base_link', rospy.Time(0))
            self.running = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.running = False

        target_x = self.target.position.x
        target_y = self.target.position.y

        if((not self.running) or (target_x == 0 and target_y == 0)):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.pub.publish(self.twist)
            return

        #x = trans.transform.translation.x
        # y = trans.transform.translation.y
        x = bot.transform.translation.x
        y = bot.transform.translation.y
        orient_q = bot.transform.rotation
        q = Quaternion(orient_q.w, orient_q.x, orient_q.y, orient_q.z)
        e = q.to_euler(degrees=True)
        currYaw = e[2]
        # calculate orientation offset to destination
        # tan^-1((nextY-currY)/(nextX-currX))
        # goalYaw = math.degrees(math.atan2(y, x))
        goalYaw = math.degrees(math.atan2(target_y - y, target_x - x))
        currYaw360 = self.headingConvert(currYaw)
        goalYaw360 = self.headingConvert(goalYaw)

        yawErr = goalYaw360 - currYaw360

        if yawErr > 180:
            yawErr = yawErr - 360
        elif yawErr < -180:
            yawErr = yawErr + 360

        dist = math.sqrt((target_x-x)**2+(target_y-y)**2)

        if dist > self.DEST_TOL:
            #angular = self.K_HDG*goalYaw
            angular = self.K_HDG*yawErr
            #if abs(goalYaw) < self.HDG_TOL:
            if abs(yawErr) < self.HDG_TOL:
                linear = .25
            else:
                linear = 0
        else:
            angular = 0
            linear = 0

        # limits bounding
        if angular > 1.5 :
            angular = 1.5
        elif (angular < -1.5) :
            angular = -1.5
        elif (0 < angular and angular < 1 and linear == 0):
            angular = 1
        elif(-1 < angular and angular < 0 and linear == 0):
            angular = -1

        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.pub.publish(self.twist)

    # Handler that publishes the linear x and 
    # angular z values that are sent to drive the TI Bot
    # the TI Bot from roadrunner
    # Topic: Cmd_vel
    # Msg type: Twist
    # Frequency: 100 Hz
    def handler(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('controller', anonymous = True)

    c = Controller()
    #c.handler()
    rospy.spin()
