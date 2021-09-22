#!/usr/bin/env python

# Robot.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: This file contains the Robot class and constants

# Imports
import time
import math
import numpy as np
from geometry_msgs.msg import Point, Pose

# Global Constants
ROBOT_W = 0.1   # (m)
ROBOT_L = 0.15  # (m)

class Robot:
    """Robots"""
    def  __init__(self):
        self.ID = self.getID()
        self.pose = self.getPose()
        self.dest = self.getDest()
        self.status = self.getStatus()

    def getID():
        return 0

    def getPose(): # TODO: Get Pose
        return Pose(0, 0, 0) # x, y, theta
    
    def getDest(): # TODO: Get Dest
        return Point(0, 0, 0) # x, y, theta

    def getStatus(): # TODO: Get Status
        return "Finshed" # "Running" "Finished" or 