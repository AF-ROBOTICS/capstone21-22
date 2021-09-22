#!/usr/bin/env python

# Field.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: This file contains the Field class and constants

# Imports
import time
import math
import numpy as np
from geometry_msgs.msg import Point, Pose
from robot import Robot

# Global Variables
FIELD_X = 5     # (m)
FIELD_Y = 5.5   # (m)

class Field:
    def __init__(self):
        #TODO: Make the Field Effective