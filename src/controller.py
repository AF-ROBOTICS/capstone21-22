#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose, Point, Twist

from squaternion import Quaternion

## Controller
#
#  Subscribes to usafabot_Bot's current position (from Roadrunner) and destination 
#   position (from master node) to determine linear and angular velocity and
#   publishes those values to the usafabot_Bot
#
#  Subscriber
#   Topic: usafabot_curr_pos
#     Msg type: Pose
#     Freq: 100 Hz
#   Topic: usafabot_dest_pos
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
        
        self.ctrl_c = False
        
        rospy.on_shutdown(self.shutdownhook)

        rospy.Subscriber('curr_pos', Pose, self.callback_CurrPos)
        rospy.Subscriber('dest_pos', Point, self.callback_DestPos)
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

        rospy.Timer(rospy.Duration(.01), self.callback_converter)
        


    # Subscribe function that gets current position of 
    # the TI Bot from roadrunner
    # Topic: usafabot_Curr_Pos
    # Msg type: Pose
    def callback_CurrPos(self, data):
        self.currX = data.position.x
        self.currY = data.position.y
       
        q = Quaternion(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        e = q.to_euler(degrees=True)
        self.currYaw = e[2]
    
    # Subscribe function that gets destination position of 
    # the TI Bot from controller
    # Topic: usafabot_Dest_Pos
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
    # the usafabot_Bot to the destination. Provides a linear/angular velocity
    # to usafabot_Bot using a proportional controller based on the orientation offset
    # Frequency: 100 Hz
    def callback_converter(self, event):  
        if not self.ctrl_c:
            if(self.nextX == 0 and self.nextY ==0):
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.pub.publish(self.twist)
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
                            
            linear = 0
            angular = 0
            
            # TODO: Update to PID controller
            # Porportional controller that updates angular and linear velocity
            # of the usafabot until within distance tolerance
            if dist > self.DEST_TOL :
                angular = self.K_HDG * yawErr
                if abs(yawErr) < self.HDG_TOL :
                    linear = .5
                else :
                    linear = 0

            # limits bounding
            if angular > 2 :
                angular = 2
            elif (angular < -2) :
                angular = -2

            # TODO: remove this line when moving to real robot; 
            # this is used because the simulated robot moves clockwise with a 
            # positive angular vel while real robot moves counterclockwise
            #angular = -angular
            
            self.twist.linear.x = linear
            self.twist.angular.z = angular
            self.pub.publish(self.twist)
        
    def shutdownhook(self):
        self.ctrl_c = True
        rospy.loginfo("Shutting down controller and stopping USAFABOT")
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.pub.publish(self.twist)
    
if __name__ == '__main__':
    rospy.init_node('controller', anonymous = True)

    Controller()
    rospy.spin()
