#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

rospy.init_node('talker', anonymous=True)
pub0 = rospy.Publisher('/usafabot0/dest_pos', Point, queue_size=1)
dest0 = Point(1.0, 1.0, 0.0)
pub1 = rospy.Publisher('/usafabot1/dest_pos', Point, queue_size=1)
dest1 = Point(2.0, 2.0, 0.0)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub0.publish(dest0)
    pub1.publish(dest1)
    rate.sleep()
