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
# | FILENAME      : routing.py
# | AUTHOR(S)     : C1C Cason Couch
# | CREATED       : 5 Novermber 2021
# | Last Update   : 06 Apr 2022
"""This file contains various working designs for routing codes. 
The routing codes accept information on the current positions of all bots and can be interchanged within a master file."""

# Imports
from util import Action, manhattan_distance

#####################################################
#                Routing Algorithms                 #
#####################################################

# # Greedy Search Algorithm: Status-Incomplete
# def greedy(field, robots):
#     """Greedy Search Algorithm for one robot timestep"""
    
#     # Array of Arrays Possible Timesteps
#     timesteps = []

#     # Actions Array
#     actions = [Action(0), Action(1), Action(2), Action(3), Action(4)]

#     # heck Heuristic with All Possible Timesteps
#     for i in range(25^5):
#         timestep = []
#         timestep.append(Action())
#         timesteps.append

#     # Find Most Productive Timestep
#     while not :
        
#         # Cost associated with a timestep
#         cost = 0

#         # Dequeue a State off the Priority Queue
#         current = states.pop()

#         # Check if Current has been Visited Before
#         if not current[0] in visited:
#             # Add Current to Visited List
#             visited.append(current[0])

#             # Check if Current is a Goal State
#             if field.final_formation(current[0]):
#                 return current[1]

#             # Enqueue Successors to Priority Queue
#             successors = robot.getSuccessors(current[0])
#             for successor in successors:
#                 temp = current[1].copy()
#                 temp.append(successor[1])
#                 states.push((successor[0], temp, current[2] + successor[2]), current[2] + successor[2] + heuristic(successor[0], problem))

#     # If Goal not Found Return Empty List
#     return []

# Single-Step Greedy: Status-Working!
def single_greedy(field, field_next, robot):
    """Evaluates one iteration of a greedy search using previous robot positions"""
    
    # Distances Array
    dist = []

    # Declare Potential Next Positions
    x = [robot.pos_cell[0], robot.pos_cell[0] + 1, robot.pos_cell[0]    , robot.pos_cell[0] - 1, robot.pos_cell[0]    ]
    y = [robot.pos_cell[1], robot.pos_cell[1]    , robot.pos_cell[1] + 1, robot.pos_cell[1]    , robot.pos_cell[1] - 1]

    # Loop Through All Actions
    for i in range(len(x)):
        # Find Distance from Cell to Goal
        dist.append( manhattan_distance(x[i], y[i], robot.dest_cell[0], robot.dest_cell[1]) )
        
        # Check if Cell is Occupied
        if field_next.cells[x[i]][y[i]].robot:
            dist[i] += 999999

        # Check if Cell was Occupied
        if field.cells[x[i]][y[i]].robot:
            dist[i] += 1

        # Check if Robot Stayed On Previous Iterations
        if i == 0 and not robot.status == 2:
            dist[i] += robot.stay_count

    # Find Ideal Action
    i = dist.index(min(dist))
    action = Action(i)

    # Update Stay Count
    if i == 0:
        robot.stay_count += 1
    else:
        robot.stay_count = 0

    # Map Bot to Next Field
    field_next.cells[x[i]][y[i]].robot = robot

    # If Bot is Finished, Update It
    if robot.pos_cell[0] == robot.dest_cell[0] and robot.pos_cell[1] == robot.dest_cell[1]:
        robot.status = 2

    # Return Next Step
    return (x[i], y[i])

# # A*: Status-Incomplete
# def a_star(field, robot):
#     """A* Search Algorithm for routing robots"""

#     # Array of Arrays holding States
#     path = []
    
#     # Cost associated with a path
#     cost = 0
    
#     # Create an Empty Visited List: (cell, robot)
#     visited = []

#     # State is a Field, Path, and Cost
#     state = (field, path, cost)

#     # Declare Priority Queue and Populate with Start State
#     states = util.PriorityQueue()
#     states.push(state, field.manhattan())

#     # Run Loop Until Priority Queue is Empty or Goal is Found
#     while not states.isEmpty():

#         # Dequeue a State off the Priority Queue
#         current = states.pop()

#         # Check if Current has been Visited Before
#         if not current[0] in visited:
#             # Add Current to Visited List
#             visited.append(current[0])

#             # Check if Current is a Goal State
#             if field.final_formation(current[0]):
#                 return current[1]

#             # Enqueue Successors to Priority Queue
#             successors = robot.getSuccessors(current[0])
#             for successor in successors:
#                 temp = current[1].copy()
#                 temp.append(successor[1])
#                 states.push((successor[0], temp, current[2] + successor[2]), current[2] + successor[2] + heuristic(successor[0], problem))

#     # If Goal not Found Return Empty List
#     return []