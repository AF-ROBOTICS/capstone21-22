#!/usr/bin/env python

# Robot.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch, Anthony Tolbert
# Description: This file contains the Robot class and constants

# Imports
import time
import math
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
from routing import Action

# Global Constants
ROBOT_W = 0.1   # (m)
ROBOT_L = 0.15  # (m)
DEST_TOLERANCE = (ROBOT_W**2 + ROBOT_L**2)**.5 # (m) temporary until we figure out actual distance


class Robot:
    """Robot objects have an ID, position, destination, and status"""

    def __init__(self, USAFABOT):
        self.ID = self.getID()
        # self.pose = Ground_Pose(None)
        # self.dest = self.getDest()
        self.status = self.getStatus()
        self.action = self.getAction()
        self.curr_pos = Ground_Pose()
        self.dest_pos = Point()
        self.name = USAFABOT
        # Ros Publisher
        self.pub = rospy.Publisher(self.name + '/dest_pos', Point, queue_size = 10)
        # Ros Listener
        rospy.Subscriber(self.name + '/curr_pos', Pose, self.callback_currPos)

    def getID(self):
        return self.name
        
    def callback_currPos(self, data): # Updates current position by listening to rosTopic
        self.curr_pos.x = round(data.position.x, 3)
        self.curr_pos.y = round(data.position.y, 3)
        self.curr_pos.theta = round(data.orientation.z, 3)

    # def getPose(): # TODO: Get Pose
        # return Ground_Pose(None)
    
    def getDest(self):  # Returns current destination position as set by setDest()
        return self.dest_pos

    def getStatus(self): # simple; returns if at dest or not
        if (abs(self.curr_pos.x - self.dest_pos.x) < DEST_TOLERANCE) and (abs(self.curr_pos.y - self.dest_pos.y) < DEST_TOLERANCE):
            return "Finished"
        else:
            return "Running"
        # return None # "Running" "Finished" or
        
    def setDest(self, dest_point): # changed from (x,y) to single object, Can change back
        self.dest_pos.x = dest_point.x
        self.dest_pos.y = dest_point.y
    
    def getAction(): # TODO: Get Action
        return Action(None)
    
class Ground_Pose:
    def __init__(self):
        """Our ground bots need only x, y, and theta for navigation"""
        self.x = float(None)
        self.y = float(None)
        self.theta = float(None)
