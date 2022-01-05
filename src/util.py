#!/usr/bin/env python

# routing.py - Robotic Teaming Artificial Intelligence
# Author: Cason Couch
# Description: This file contains the Robot class

import heapq

# Global Variables
FIELD_W = 5.5   # (m) x-axis
FIELD_L = 5     # (m) y-axis
CELL_W  = 0.2   # (m) 

FIELD_X = FIELD_W // CELL_W + 1 # Discrete Indices
FIELD_Y = FIELD_L // CELL_W + 1

ROBOT_W = 0.1   # (m)
ROBOT_L = 0.15  # (m)
DEST_TOLERANCE = (ROBOT_W**2 + ROBOT_L**2)**.5 # (m) temporary until we figure out actual distance

# Enumerates Actions for use in AI
class Action(Enum):
    """Actions for robots to take"""
    STAY = 0
    RIGHT = 1
    UP = 2
    LEFT = 3
    DOWN = 4

# Cells in Field
class Cell:
    """Cell objects can contain a robot's position or destination"""
    def __init__(self):
        self.robot  = None
        # self.cardinal = Cardinal(None) # TODO
        self.dest   = None

# class Sub_Action(Enum):
#     """Sub-actions for robot movemnts"""
#     STAY = 0
#     FORW_1 = 1
#     BACK_1 = 2
#     RIGH_90 = 3
#     LEFT_90 = 4

# Robot Status
class Status(Enum):
    """Status indicates the task completion status of a robot"""
    START = 0
    RUNNING = 1
    FINISHED = 2

class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def is_empty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)

class PriorityQueueWithFunction(PriorityQueue):
    """
    Implements a priority queue with the same push/pop signature of the
    Queue and the Stack classes. This is designed for drop-in replacement for
    those two classes. The caller has to provide a priority function, which
    extracts each item's priority.
    """
    def  __init__(self, priorityFunction):
        "priorityFunction (item) -> priority"
        self.priorityFunction = priorityFunction      # store the priority function
        PriorityQueue.__init__(self)        # super-class initializer

    def push(self, item):
        "Adds an item to the queue with priority from the priority function"
        PriorityQueue.push(self, item, self.priorityFunction(item))


def manhattan_distance(x1, y1, x2, y2):
    "Returns the Manhattan distance between two cells"
    return abs(x1 - x2) + abs(y1 - y2)