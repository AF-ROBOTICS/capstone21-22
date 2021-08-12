#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

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
    HDG_TOL = 15
    # destination tolerance +/- meters
    DEST_TOL = 0.05
    # roation controller constant
    K_HDG = 0.1
    
    def __init__(self):
        # instance variables unique to each instance of class
        self.currX = 0
        self.nextX = 0
        self.currY = 0
        self.nextY = 0
        self.currYaw = 0
        self.twist = Twist()
        
        # Frequency of publisher - 100 Hz
        self.rate = rospy.Rate(100)

        rospy.Subscriber('ti_curr_pos', Pose, self.callback_CurrPos)
        rospy.Subscriber('ti_dest_pos', Point, self.callback_DestPos)
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

        rospy.Timer(rospy.Duration(.01), self.callback_converter)

    # Subscribe function that gets current position of 
    # the TI Bot from roadrunner
    # Topic: TI_Curr_Pos
    # Msg type: Pose
    def callback_CurrPos(self, data):
        self.currX = data.position.x
        self.currY = data.position.y
        self.currYaw = data.orientation.z
    
    # Subscribe function that gets destination position of 
    # the TI Bot from controller
    # Topic: TI_Dest_Pos
    # Msg type: Point
    def callback_DestPos(self, data):
        self.nextX = data.x
        self.nextY = data.y

    # The Roadrunner provides the orientation from 0 to 180 deg and 
    # 0 to -180 deg
    # This function converts orientation to 0 to 360 deg (with 0 deg heading at 
    # the positive x direction)
    def headingConvert(self, yaw):
        if yaw < 0 :
            return 360 + yaw
        else :
            return yaw

    # Callback function that calculates orientation offset needed to navigate
    # the TI_Bot to the destination. Provides a linear/angular velocity
    # to TI_Bot using a proportional controller based on the orientation offset
    # Frequency: 100 Hz
    def callback_converter(self, event):  
        if(self.nextX == 0 and self.nextY ==0):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            return
        goalYaw = 0
        yawErr = 0
        currYaw360 = 0
        goalYaw360 = 0
        dist = 0
        
        # calculate orientation offset to destination
        # tan^-1((nextY-currY)/(nextX-currX))
        goalYaw = math.degrees(math.atan2(self.nextY - self.currY, 
                                            self.nextX - self.currX))

        # Current yaw provided between 0 to +-180 deg
        # Convert both current and goal headings to 0 to 360 deg
        currYaw360 = self.headingConvert(self.currYaw)
        goalYaw360 = self.headingConvert(goalYaw)

        yawErr = goalYaw360 - currYaw360

        # determine if the robot should move clockwise or counterclockwise
        if yawErr > 180 :
            yawErr = yawErr - 360
        elif yawErr < -180 :
            yawErr = yawErr + 360
        
        # determine how far TI Bot is from the goal
        dist = math.sqrt((self.nextY - self.currY)**2 
                        + (self.nextX - self.currX)**2)
        xIn = 0
        zIn = 0
        
        # TODO: Update to PID controller
        # Porportional controller that updates angular and linear velocity
        # of the TI_Bot until within distance tolerance
        if dist > self.DEST_TOL :
            zIn = self.K_HDG * yawErr
            if abs(yawErr) < self.HDG_TOL :
                xIn = .5
            else :
                xIn = 0

        # limits bounding
        if zIn > 6 :
            zIn = 6
        elif (zIn < -6) :
            zIn = -6
        
        self.twist.linear.x = xIn
        self.twist.angular.z = zIn
        
    # Handler that publishes the linear x and 
    # angular z values that are sent to drive the TI Bot
    # the TI Bot from roadrunner
    # Topic: Cmd_vel
    # Msg type: Twist
    # Frequency: 100 Hz
    def handler(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist)
            self.rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('controller', anonymous = True)

    c = Controller()
    c.handler()
