#!/usr/bin/env python

# Field.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: Contains the Field class

# Imports
import math
from robot import Robot
from enum import Enum
import util
from util import FIELD_X, FIELD_Y, FIELD_W, FIELD_L, CELL_W, Action, manhattan_distance
from cell import Cell

class Field:
    """Field contains a grid of Cell objects which map robots and their destinations"""
    
    def __init__(self):
        """Field objects contain an array of Robot objects, Grid of Cells, and array for lost robots"""
        self.cells = Cell[math.ceil(FIELD_W/CELL_W)][math.ceil(FIELD_L/CELL_W)]
        self.lost_bots = Robot[None]
    
    def map_bots(self, robots):
        """Populates field cells with current robot positions"""

        # Clear All Cells
        for cell in self.cells.asList():
            cell.robot = Robot(None)

        # Reset Lost Bot List
        self.lost_bots = Robot[None]

        # Add All Robots to Respective Cells
        for robot in robots:
            x = robot.pos.x // CELL_W
            y = robot.pos.y // CELL_W
            if 0 >= x < FIELD_X and 0 >= y < FIELD_Y:
                self.cells[x][y].robot = robot
                robot.pos_cell = (x, y)
            else:
                self.lost_bots.append(robot)
                robot.pos_cell = (None)

    def map_dest(self, robots):
        """Populates field cells with current destination positions"""

        # Clear All Cells
        for cell in self.cells.asList():
            cell.dest = Robot(None)

        # Add All Robots to Respective Cells
        for robot in robots:
            x = math.floor(robot.dest.x / CELL_W)
            y = math.floor(robot.dest.y / CELL_W)
            self.cells[x][y].dest = robot
            robot.dest_cell = (x, y)

    # TODO
    # # Cleanup: Centers Robots in Cells and Orients at Right Angles
    # def cleanup(self):
    #     """Moves bots to cell locations and places lost bots back in field"""
    #     for robot in self.robots:
    #         break
    #     return

    # Determines if All Robots are in Destination Positions
    def final_formation(self):
        for cell in self.cells.asList():
            if cell.robot != cell.dest:
                return False
        return True

    def manhattan(self, robots):
        manhattan = 0
        for robot in robots:
            manhattan = util.manhattan_distance(robot.pos_cell[0], robot.pos_cell[1], robot.dest_cell[0], robot.dest_cell[1])
        return manhattan

# Greedy Search Algorithm
def greedy(field, robots):
    """Greedy Search Algorithm for one robot timestep"""
    
    # Array of Arrays Possible Timesteps
    timesteps = []

    # Actions Array
    actions = [Action(0), Action(1), Action(2), Action(3), Action(4)]

    # heck Heuristic with All Possible Timesteps
    for i in range(25^5):
        timestep = []
        timestep.append(Action())
        timesteps.append

    # Find Most Productive Timestep
    while not :
        
        # Cost associated with a timestep
        cost = 0

        # Dequeue a State off the Priority Queue
        current = states.pop()

        # Check if Current has been Visited Before
        if not current[0] in visited:
            # Add Current to Visited List
            visited.append(current[0])

            # Check if Current is a Goal State
            if field.final_formation(current[0]):
                return current[1]

            # Enqueue Successors to Priority Queue
            successors = robot.getSuccessors(current[0])
            for successor in successors:
                temp = current[1].copy()
                temp.append(successor[1])
                states.push((successor[0], temp, current[2] + successor[2]), current[2] + successor[2] + heuristic(successor[0], problem))

    # If Goal not Found Return Empty List
    return []

# A*: A-Star Search Algorithm for Routing
def single_greedy(field, field_next, robot):
    """Greedy Search Algorithm for routing robot"""

    # Distances Array
    dist = []

    # New Positions
    x = [robot.pos_cell(0), robot.pos_cell(0) + 1, robot.pos_cell(0), robot.pos_cell(0) - 1, robot.pos_cell(0)]
    y = [robot.pos_cell(1), robot.pos_cell(1), robot.pos_cell(1) + 1, robot.pos_cell(1), robot.pos_cell(1) - 1]

    # Loop Through All Actions
    for i in range(x):
        # Find Distance from Cell to Goal
        dist[i] = util.manhattan_distance(x[i], y[i], robot.dest_cell(0), robot.dest_cell(1))
        
        # Check if Cell is Occupied
        if field_next.cells[x[i]][y[i]]:
            dist[i] += 999999

    # Find Ideal Action
    i = dist.index(min(dist))
    action = Action(i)

    # Map Bot to Next Field
    field_next.cells[x[i]][y[i]].robot = robot

    return (x[i], y[i])

# A*: A-Star Search Algorithm for Routing
def a_star(field, robot):
    """A* Search Algorithm for routing robots"""

    # Array of Arrays holding States
    path = []
    
    # Cost associated with a path
    cost = 0
    
    # Create an Empty Visited List: (cell, robot)
    visited = []

    # State is a Field, Path, and Cost
    state = (field, path, cost)

    # Declare Priority Queue and Populate with Start State
    states = util.PriorityQueue()
    states.push(state, field.manhattan())

    # Run Loop Until Priority Queue is Empty or Goal is Found
    while not states.isEmpty():

        # Dequeue a State off the Priority Queue
        current = states.pop()

        # Check if Current has been Visited Before
        if not current[0] in visited:
            # Add Current to Visited List
            visited.append(current[0])

            # Check if Current is a Goal State
            if field.final_formation(current[0]):
                return current[1]

            # Enqueue Successors to Priority Queue
            successors = robot.getSuccessors(current[0])
            for successor in successors:
                temp = current[1].copy()
                temp.append(successor[1])
                states.push((successor[0], temp, current[2] + successor[2]), current[2] + successor[2] + heuristic(successor[0], problem))

    # If Goal not Found Return Empty List
    return []

# class Sub_Action(Enum):
#     STAY = 0
#     FORW_1 = 1
#     BACK_1 = 2
#     RIGH_90 = 3
#     LEFT_90 = 4
