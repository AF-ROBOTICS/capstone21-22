#!/usr/bin/env python

# Field.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: Contains the Field class

# Imports
import math
from robot import Robot
from util import FIELD_X, FIELD_Y, FIELD_W, FIELD_L, CELL_W, Action, manhattan_distance, Cell

class Field:
    """Field contains a grid of Cell objects which map robots and their destinations"""
    
    def __init__(self):
        """Field objects contain an array of Robot objects, Grid of Cells, and array for lost robots"""
        self.cells = []
        for x in range(math.ceil(FIELD_W/CELL_W)):
            cell_row = []
            for y in range(math.ceil(FIELD_L/CELL_W)):
                cell_row.append(Cell())
            self.cells.append(cell_row)
        self.lost_bots = []

    def map_bots(self, robots):
        """Populates field cells with current robot positions"""

        # Clear All Cells
        for cell_row in self.cells:
            for cell in cell_row:
                cell.robot = None

        # Reset Lost Bot List
        self.lost_bots = []

        # Add All Robots to Respective Cells
        for robot in robots:
            x = int(robot.pos.x // CELL_W)
            y = int(robot.pos.y // CELL_W)
            if 0 >= x < FIELD_X and 0 >= y < FIELD_Y:
                self.cells[x][y].robot = robot
                robot.pos_cell = (x, y)
            else:
                self.lost_bots.append(robot)
                robot.pos_cell = (None, None)

    def map_dest(self, robots):
        """Populates field cells with current destination positions"""

        # Clear All Cells
        for cell_row in self.cells:
            for cell in cell_row:
                cell.robot = None

        # Add All Robots to Respective Cells
        for robot in robots:
            x = math.floor(robot.dest.x / CELL_W)
            y = math.floor(robot.dest.y / CELL_W)
            self.cells[x][y].dest = robot
            robot.dest_cell = (x, y)

    # TODO: Cleanup Function
    # # Cleanup: Centers Robots in Cells and Orients at Right Angles
    # def cleanup(self):
    #     """Moves bots to cell locations and places lost bots back in field"""
    #     for robot in self.robots:
    #         break
    #     return

    def final_formation(self):
        """Returns true if all robots are at destinations"""
        for cell in self.cells.asList():
            if cell.robot != cell.dest:
                return False
        return True

    def manhattan(self, robots):
        """Returns total manhattan distance of all robots to destination positions"""
        manhattan = 0
        for robot in robots:
            manhattan = manhattan_distance(robot.pos_cell[0], robot.pos_cell[1], robot.dest_cell[0], robot.dest_cell[1])
        return manhattan


