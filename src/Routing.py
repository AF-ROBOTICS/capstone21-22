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


# Cleanup: Centers Robots in Cells and Orients at Right Angles
def cleanup(field: Field):
    # TODO: Create Cleanup
    return

def evaluate_action(field: Field, action: Action):
    return

# a*: A-Star Search Algorithm for Routing
def aStarSearch(field: Field): # FIXME: This is CS471 A*
    """CS 471 PEX 1 Question 4"""

    # Declare Priority Queue and Populate with Start State
    states = util.PriorityQueue()
    states.push((field, [], 1), heuristic(field))

    # Create an Empty Visited List
    visited = []

    # Run Loop Until Priority Queue is Empty or Goal is Found
    while not states.isEmpty():

        # Dequeue a State off the Priority Queue
        current = states.pop()

        # Check if Current has been Visited Before
        if not current[0] in visited:
            # Add Current to Visited List
            visited.append(current[0])

            # Check if Current is a Goal State
            if final_formation(current[0]):
                return current[1]

            # Enqueue Successors to Priority Queue
            successors = problem.getSuccessors(current[0])
            for successor in successors:
                temp = current[1].copy()
                temp.append(successor[1])
                states.push((successor[0], temp, current[2] + successor[2]), current[2] + successor[2] + heuristic(successor[0], problem))

    # If Goal not Found Return Empty List
    return []

def heuristic(field: Field):
    manhattan = 0
    for robot in field.robots:
        manhattan = util.manhattan_distance(robot.pos_cell[0], robot.pos_cell[1], robot.dest_cell[0], robot.dest_cell[1])
    return manhattan

def final_formation(field: Field):
    for cell in field.cells.asList():
        if cell.robot != cell.dest:
            return False
    return True

# This implementation is unused
# def drive(robots: Robot[25]):
#     """Creates drive commands from Action Decisons"""
#     for robot in robots:
#         if robot.action is Action(0): # FIXME: Stay
#             # TODO: Implement Stay
#             break
#         elif robot.action is Action(1): # FIXME: FORW_1
#             # TODO: Implement FORW
#             break
#         elif robot.action is Action(2): # FIXME: BACK_1
#             # TODO: Implement FORW
#             break
#         elif robot.action is Action(3): # FIXME: RIGH_90
#             # TODO: Implement FORW
#             break
#         elif robot.action is Action(4): # FIXME: LEFT_90
#             # TODO: Implement FORW
#             break
