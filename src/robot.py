#!/usr/bin/env python

# +----------------------------------------------------------------------------
# |
# | United States Air Force Academy     __  _______ ___    _________
# | Dept of Electrical &               / / / / ___//   |  / ____/   |
# | Computer Engineering              / / / /\__ \/ /| | / /_  / /| |
# | 2354 Fairchild Drive Ste 2F6     / /_/ /___/ / ___ |/ __/ / ___ |
# | USAF Academy, CO 80840           \____//____/_/  |_/_/   /_/  |_|
# |
# | ---------------------------------------------------------------------------
# |
# | FILENAME      : robot.py
# | AUTHOR(S)     : C1C Cason Couch
# | CREATED       : 5 Novermber 2021
# | Last Update   : 06 Apr 2022
"""This file contains the Robot class and its class functions.
It also contains the Ground_Pose class which is used for some special position messages."""

# Imports
import time
import math
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
from routing import Action
from util import DEST_TOLERANCE, Status

class Robot:
    """Robot objects have an ID, position, destination, and status"""

    def __init__(self, usafabot: str, dest = (0,0,0)):
        self.ID = None
        self.status = Status(0)
        self.action = self.getAction()
        self.pos = Ground_Pose()
        self.dest = Ground_Pose(dest[0], dest[1], dest[2])
        self.name = usafabot
        self.pos_cell = (None, None)
        self.dest_cell = (None, None)
        self.step_cell = (None, None)
        self.stay_count = 0
        
        # Ros Publisher
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size = 10)
        
        # Ros Listener
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)

    # -------------------------------------------------------------------------------
    # Class Functions
    # -------------------------------------------------------------------------------
    
    def callback_currPos(self, data): # Updates current position by listening to rosTopic
        self.pos.x = round(data.position.x, 3)
        self.pos.y = round(data.position.y, 3)
        self.pos.theta = round(data.orientation.z, 3)

    def updateStatus(self): # simple; returns if at dest or not
        if (abs(self.curr_pos.x - self.dest_pos.x) < DEST_TOLERANCE) and (abs(self.curr_pos.y - self.dest_pos.y) < DEST_TOLERANCE):
            return "Finished"
        else:
            return "Running"
        # return None # "Running" "Finished" or
        
    def setDest(self, dest_point, dest_theta = 0): # changed from (x,y) to single object, Can change back
        self.dest.x = dest_point[0]
        self.dest.y = dest_point[1]
        self.dest.theta = dest_theta
        
    def getAction(self):
        return None

    # def get_successors(self):
    #     """Returns array of successor points: [(x,y)]"""
    #     successors = []
    #     if 
    
class Ground_Pose:
    def __init__(self, x = -1, y = -1, theta = -1):
        """Our ground bots need only x, y, and theta for navigation"""
        # float(None) threw an error. -1 represents init/error vals
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
