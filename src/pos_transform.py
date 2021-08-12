#!/usr/bin/env python3
import rospy
import math
from squaternion import Quaternion # requires squaternion and attrs library

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


# Position
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
class Position:

    def __init__(self):
        # instance variables unique to each instance of class
        self.EPose = Pose()
        self.Qx = 0
        self.Qy = 0
        self.Qz = 0
        self.Qw = 0

        rospy.Subscriber('odom', Odometry, self.callback_CurrPos)
        self.pub = rospy.Publisher('ti_curr_pos', Pose, queue_size=1)

    # Subscribe function that gets current position of
    # the TI Bot from Gazebo and transforms to Euler
    # Publishes new position over ti_curr_pos topic
    # Topic:
    #   Subscriber: odom
    #   Publisher: ti_cur_pos
    # Msg type:
    #   Subscriber: Odometry
    #   Publisher: Pose
    def callback_CurrPos(self, data):
        self.EPose.position.x = data.pose.pose.position.x
        self.EPose.position.y = data.pose.pose.position.y
        self.Qx = data.pose.pose.orientation.x
        self.Qy = data.pose.pose.orientation.y
        self.Qz = data.pose.pose.orientation.z
        self.Qw = data.pose.pose.orientation.w

        # math magic
        q = Quaternion(self.Qw, self.Qx, self.Qy, self.Qz)
        e = q.to_euler(degrees=True)
        self.EPose.orientation.z = e[2] - 90

        if self.EPose.orientation.z < -180:
            self.EPose.orientation.z += 360
            

        self.pub.publish(self.EPose)


if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)

    p = Position()
    rospy.spin()
