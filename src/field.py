#!/usr/bin/env python

# Field.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: Contains the Field and Cell classes

# Imports
import math
from robot import Robot
from enum import Enum

# Global Variables
FIELD_W = 5.5   # (m) x-axis
FIELD_L = 5     # (m) y-axis
CELL_W  = 0.2   # (m) 

class Field:
    def __init__(self):
        """Field contains a grid of Cell objects which map robots and their destinations"""
        
        self.cells = Cell[math.ceil(FIELD_W/CELL_W)][math.ceil(FIELD_L/CELL_W)]
        self.lost_bots = Robot[None]
    
    def map_bots(self, robots: Robot[25]):
        """Populates field cells with current robot positions"""

        # Clear All Cells
        for cell in self.cells.asList():
            cell.robot = Robot(None)

        # Reset Lost Bot List
        self.lost_bots = Robot[None]

        # Add All Robots to Respective Cells
        for robot in robots:
            x = math.floor(robot.pos.x / CELL_W)
            y = math.floor(robot.pos.y / CELL_W)
            if 0 >= x < math.ceil(FIELD_W/CELL_W) and 0 >= y < math.ceil(FIELD_L/CELL_W):
                self.cells[x][y].robot = robot
            else:
                self.lost_bots.append(robot)

    def map_dest(self, robots: Robot[25]):
        """Populates field cells with current destination positions"""

        # Clear All Cells
        for cell in self.cells.asList():
            cell.dest = Robot(None)

        # Add All Robots to Respective Cells
        for robot in robots:
            x = math.floor(robot.dest.x / CELL_W)
            y = math.floor(robot.dest.y / CELL_W)
            self.cells[x][y].dest = robot

class Cell:
    def __init__(self):
        """Cell objects can contain a robot's position or destination"""
        
        self.robot  = Robot(None)
        self.cardinal = Cardinal(None)
        self.dest   = Robot(None)

class Cardinal(Enum):
    E = 0
    N = 1
    W = 2
    S = 3
    