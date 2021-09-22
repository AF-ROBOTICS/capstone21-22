#!/usr/bin/env python

# Global_Env.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: This file creates a global environment mapping of the positions
#   and orientations of all robots in the environment and any detected obstacles
# Subscriptions: rostopic '/global_pos' (from Master.py)
# Publications: rostopic '/global_env' (to Routing.py)

# Imports
import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import Point, Pose
from robot import Robot
from field import Field

# Mapping
def mapping(robots: Robot[25]):
    field = Field()
    # TODO: Actually Add a Mapping System
    return (field)