#!/usr/bin/env python3
import rospy
import sys
import time
import logging
import csv
import socket
import math

from cflib import crazyflie, crtp
from cflib.crazyflie.log import LogConfig
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

import tf2_ros
import tf2_msgs.msg

from squaternion import Quaternion

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

## Roadrunner
#
#  Connects to a Bitcraze Roadrunner with given URI and publishes
#   current x, y, and z position and yaw orientation.
#
#  Publisher
#   Topic: ti_curr_pos
#     Msg type: Pose
#     Freq: 100 Hz
class Roadrunner:
    PERIOD = 10 # Control period. [ms]
    # heading tolerance +/- degrees
    HDG_TOL = 25
    # destination tolerance +/- meters
    DEST_TOL = 0.05
    # roation controller constant
    K_HDG = 0.01

    #def __init__(self, URI, filename):
    def __init__(self, URI, name):
        # instance variables unique to each instance of class
        self.rr_pose = Pose()
        self.rr_vel = Twist()
        self.rr = crazyflie.Crazyflie(rw_cache='./cache')

        self.name = name
        self.rate = rospy.Rate(100)
        
        self.state = JointState()
        
        # values used to determine linear and angular velocity
        self.v_l = 0
        self.v_r = 0
        
        # Connect some callbacks from the Crazyflie API
        self.rr.connected.add_callback(self._connected)
        self.rr.disconnected.add_callback(self._disconnected)
        self.rr.connection_failed.add_callback(self._connection_failed)
        self.rr.connection_lost.add_callback(self._connection_lost)
        
        print('Connecting to', URI)    
        self.rr.open_link(URI)

        self.br = tf2_ros.TransformBroadcaster()
        self.tcf = TransformStamped()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)


        self.target = Pose()
        self.twist = Twist()

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        rospy.Subscriber('/target_avg', Pose, self.callback_target)

        rospy.Timer(rospy.Duration(.01), self.callback_converter)

    def callback_target(self, data):
        self.target = data

    def headingConvert(self, yaw):
        if yaw < 0:
            return 360 + yaw
        else:
            return yaw


    #-------------------------------------------------------------------------
    # CrazyFlie Callbacks and Functions
    #-------------------------------------------------------------------------
    def _connected(self, link_uri):
        print('Connected to', link_uri)
        
        print('Waiting for position estimate to be good enough...')
        self.reset_estimator()
        
        log_orient = LogConfig(name='Kalman Orientation', period_in_ms=self.PERIOD)
        log_orient.add_variable('kalman.q0', 'float')
        log_orient.add_variable('kalman.q1', 'float')
        log_orient.add_variable('kalman.q2', 'float')
        log_orient.add_variable('kalman.q3', 'float')
        
        log_pos = LogConfig(name='Kalman Position', period_in_ms=self.PERIOD)
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        
        log_vel = LogConfig(name='Kalman Velocity', period_in_ms=self.PERIOD)
        log_vel.add_variable('kalman.statePX', 'float')
        log_vel.add_variable('kalman.statePY', 'float')
        log_vel.add_variable('kalman.statePZ', 'float')
        log_vel.add_variable('gyro.z', 'float')

        try:
            self.rr.log.add_config(log_orient)
            log_orient.data_received_cb.add_callback(self.readCurrYaw)
            log_orient.start()
            
            self.rr.log.add_config(log_pos)
            log_pos.data_received_cb.add_callback(self.readCurrPosition)
            log_pos.start()

            self.rr.log.add_config(log_vel)
            log_vel.data_received_cb.add_callback(self.readCurrVelocity)
            log_vel.start()

        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)


    def _log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def reset_estimator(self):
        self.rr.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.rr.param.set_value('kalman.resetEstimation', '0')
        # Sleep a bit, hoping that the estimator will have converged
        # Should be replaced by something that actually checks...
        time.sleep(1.5)
        
    # Callback function that reads position from Bitcraze Roadrunner
    # Frequency: 100 Hz
    def readCurrPosition(self, timestamp, data, logconf):
        self.rr_pose.position = Vector3(data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'])

    # Callback function that reads orientation from Bitcraze Roadrunner
    # Frequency: 100 Hz
    def readCurrYaw(self, timestamp, data, logconf):
        self.rr_pose.orientation.x = data['kalman.q1']
        self.rr_pose.orientation.y = data['kalman.q2']
        self.rr_pose.orientation.z = data['kalman.q3']
        self.rr_pose.orientation.w = data['kalman.q0']

    def readCurrVelocity(self, timestamp, data, logconf):
        self.rr_vel.linear = Vector3(data['kalman.statePX'], data['kalman.statePY'], data['kalman.statePZ'])
        self.rr_vel.angular = Vector3(0,0,data['gyro.z'])

    # Handler that publishes the x, y, and z position and
    # yaw orientation values from the Bitcraze Roadrunner
    # to the Controller node
    # Topic: ti_curr_pos
    # Msg type: Pose
    # Frequency: 100 Hz        
    def handler(self):
        while not rospy.is_shutdown():
            # work being done
            curr_time = rospy.Time.now()

            odom_trans = TransformStamped()
            odom_trans.header.stamp = curr_time
            odom_trans.header.frame_id = "odom"
            odom_trans.child_frame_id = "roadrunner"
            odom_trans.transform.translation = self.rr_pose.position
            odom_trans.transform.rotation = self.rr_pose.orientation
            self.br.sendTransform(odom_trans)

            odom = Odometry()
            odom.header.stamp = curr_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "roadrunner"

            odom.pose.pose = self.rr_pose
            odom.twist.twist = self.rr_vel
            self.odom_pub.publish(odom)

            self.rate.sleep()
             
        self.rr.close_link()

    def callback_converter(self, event):  
        target_x = self.target.position.x
        target_y = self.target.position.y
        bot_x = self.rr_pose.position.x
        bot_y = self.rr_pose.position.y
        q = Quaternion(self.rr_pose.orientation.w, self.rr_pose.orientation.x, self.rr_pose.orientation.y, self.rr_pose.orientation.z)
        e = q.to_euler(degrees=True)
        bot_yaw = e[2]

        if((bot_x == 0 and bot_y ==0) or (target_x == 0 and target_y == 0)):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.pub.publish(self.twist)
            return

        # calculate orientation offset to destination
        # tan^-1((nextY-currY)/(nextX-currX))
        goalYaw = math.degrees(math.atan2(target_y - bot_y, target_x - bot_x))
        currYaw360 = self.headingConvert(bot_yaw)
        goalYaw360 = self.headingConvert(goalYaw)

        print("currYaw: ", currYaw360)
        print("goalYaw: ", goalYaw360)

        yawErr = goalYaw360 - currYaw360

        if yawErr > 180:
            yawErr = yawErr - 360
        elif yawErr < -180:
            yawErr = yawErr + 360

        dist = math.sqrt((target_x-bot_x)**2+(target_y-bot_y)**2)

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

#-------------------------------------------------------------------------------
# main()
#------------------------------------------------------------------------------- 
if __name__ == "__main__":
    rospy.init_node('roadrunner', anonymous=True)
    
    crtp.init_drivers(enable_debug_driver=False, enable_serial_driver=True)

    URI_1 = 'serial://ttyAMA0'
    hostname = socket.gethostname()

    #filename = input("file:")
    #filename = str(filename)+".csv"
    #r = Roadrunner(URI_1, filename)
    r = Roadrunner(URI_1, hostname)
    r.handler()
    #r.f.close()
