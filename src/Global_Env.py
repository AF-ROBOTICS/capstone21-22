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

# Global Variables
FIELD_X = 5     # (m)
FIELD_Y = 5.5   # (m)
ROBOT_W = 0.1   # (m)
ROBOT_L = 0.15  # (m)

# Definition Class Concepts
class Robot:
    """Robots"""
    def  __init__(self):
        self.ID = self.getID()
        self.pose = self.getPose()
        self.dest = self.getDest()
        self.status = self.getStatus()

    def getID():
        return 0

    def getPose():
        return Pose(0, 0, 0)
    
    def getDest():
        return Point(0, 0, 0)

    def getStatus():
        return "Finshed"

class Field:
    """Field of Robot Operations"""

# Main
