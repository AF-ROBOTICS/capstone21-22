#!/usr/bin/env python

# Robot.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: This file contains the Robot class and constants

# Imports
import time
import math
import numpy as np
from geometry_msgs.msg import Point, Pose
from routing import Action

# Global Constants
ROBOT_W = 0.1   # (m)
ROBOT_L = 0.15  # (m)

class Robot:
    """Robot objects have an ID, position, destination, and status"""
    
    def  __init__(self):
        self.ID = self.getID()
        self.pose = Ground_Pose(None)
        self.dest = self.getDest()
        self.status = self.getStatus()
        self.action = self.getAction()

    def getID():
        return None

    def getPose(): # TODO: Get Pose
        return Ground_Pose(None)
    
    def getDest(): # TODO: Get Dest
        return Ground_Pose(None)

    def getStatus(): # TODO: Get Status
        return None # "Running" "Finished" or 
    
    def getAction(): # TODO: Get Action
        return Action(None)
    
class Ground_Pose:
    def __init__(self):
        """Our ground bots need only x, y, and theta for navigation"""
        self.x      = float(None)
        self.y      = float(None)
        self.theta  = float(None)