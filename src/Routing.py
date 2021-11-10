#!/usr/bin/env python

# Routing.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: This file maps routes for bots to take based on the
#   mapping produced in Global_Env.py
# Subscriptions: rostopic '/global_env' (from Global_Env.py)
# Publications: rostopic '/routes' (to Driver.py)

# Imports
from field import Field
from robot import Robot
from enum import Enum
import util

# Enumerates Actions for use in AI
class Action(Enum):
    STAY = 0
    RIGHT = 1
    UP = 2
    LEFT = 3
    DOWN = 4

# class Sub_Action(Enum):
#     STAY = 0
#     FORW_1 = 1
#     BACK_1 = 2
#     RIGH_90 = 3
#     LEFT_90 = 4



