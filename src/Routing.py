#!/usr/bin/env python

# Routing.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: This file maps routes for bots to take based on the
#   mapping produced in Global_Env.py
# Subscriptions: rostopic '/global_env' (from Global_Env.py)
# Publications: rostopic '/routes' (to Driver.py)

# a*
def aStarSearch(problem, heuristic=nullHeuristic):
    """CS 471 PEX 1 Question 4"""

    from game import Directions

    # Declare Priority Queue and Populate with Start State
    states = util.PriorityQueue()
    states.push((problem.getStartState(), [], 1), heuristic(problem.getStartState(), problem))

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
            if problem.isGoalState(current[0]):
                return current[1]

            # Enqueue Successors to Priority Queue
            successors = problem.getSuccessors(current[0])
            for successor in successors:
                temp = current[1].copy()
                temp.append(successor[1])
                states.push((successor[0], temp, current[2] + successor[2]), current[2] + successor[2] + heuristic(successor[0], problem))

    # If Goal not Found Return Empty List
    return []