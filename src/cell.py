#!/usr/bin/env python

# cell.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: Contains the Cell class

import robot

class Cell:
    """Cell objects can contain a robot's position or destination"""
    def __init__(self):
        self.robot  = Robot(None)
        # self.cardinal = Cardinal(None) # TODO
        self.dest   = Robot(None)

    # class Cardinal(Enum):
    #     E = 0
    #     N = 1
    #     W = 2
    #     S = 3