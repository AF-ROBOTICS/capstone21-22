# Import important libraries
import roslib
import rospy
import time
import serial
import array
import numpy as np
import time
from scipy.optimize import linear_sum_assignment

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

word_position = np.zeros((2,25))
posX = []
posY = []

def split(word):
    word = word.upper()
    return [char for char in word]

def switch_position(letter):
    #DICTIONARY FOR ALL LETTERS AND THEIR POSITIONS ON GRID
    position = {
        'A': ([0.0, 0.6, 0.15, 0.45, 0.3, 0.3],
              [0.0, 0.0, 0.50, 0.50, 1.0, 0.5]),

        'B': ([0.0, 0.0, 0.0, 0.60, 0.60, 0.40, 0.4, 0.4],
              [0.0, 0.5, 1.0, 0.25, 0.75, 0.50, 0.0, 1.0]),
 
        'C': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 1.0]),
            
        'D': ([0.0, 0.0, 0.0, 0.30, 0.30, 0.60],
              [0.0, 0.5, 1.0, 0.25, 0.75, 0.50]),
            
        'E': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.0, 1.0]),
            
        'F': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6],
              [0.0, 0.5, 1.0, 0.5, 1.0, 1.0]),
            
        'G': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6, 0.3],
              [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 1.0, 0.3, 0.3]),
            
        'H': ([0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.5, 0.0, 0.5, 1.0]),
            
        'I': ([0.0, 0.3, 0.6, 0.3, 0.0, 0.3, 0.6],
              [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]),
            
        'J': ([0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.0],
              [0.0, 0.3, 0.0, 0.5, 1.0, 1.0, 1.0]),
            
        'K': ([0.0, 0.0, 0.0, 0.30, 0.30, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.25, 0.75, 0.0, 1.0]),
            
        'L': ([0.0, 0.0, 0.0, 0.3, 0.6],
              [0.0, 0.5, 1.0, 0.0, 0.0]),
            
        'M': ([0.0, 0.0, 0.0, 0.15, 0.3, 0.45, 0.6, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.75, 0.5, 0.75, 1.0, 0.5, 0.0]),
            
        'N': ([0.0, 0.0, 0.0, 0.15, 0.3, 0.45, 0.6, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.75, 0.5, 0.25, 0.0, 0.5, 1.0]),
            
        'O': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.5, 1.0]),
            
        'P': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 1.0]),
            
        'Q': ([0.00, 0.0, 0.0, 0.20, 0.2, 0.4, 0.40, 0.4, 0.6],
              [0.25, 0.7, 1.0, 0.25, 1.0, 1.0, 0.25, 0.7, 1.0]),
            
        'R': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6, 0.45],
              [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 1.0, 0.0, 0.25]),
            
        'S': ([0.0, 0.0, 0.0, 0.00, 0.3, 0.3, 0.3, 0.6, 0.6, 0.6, 0.60],
              [0.0, 0.5, 1.0, 0.75, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.25]),
            
        'T': ([0.0, 0.3, 0.3, 0.3, 0.6],
              [1.0, 0.0, 0.5, 1.0, 1.0]),
            
        'U': ([0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.0, 0.0, 0.5, 1.0]),
            
        'V': ([0.0, 0.15, 0.3, 0.45, 0.6],
              [1.0, 0.50, 0.0, 0.50, 1.0]),
            
        'W': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6, 0.6],
              [0.0, 0.5, 1.0, 0.0, 0.5, 0.0, 0.5, 1.0]),
            
        'X': ([0.0, 0.0, 0.3, 0.6, 0.6],
              [0.0, 1.0, 0.5, 0.0, 1.0]),
            
        'Y': ([0.0, 0.3, 0.3, 0.6],
              [1.0, 0.5, 0.0, 0.5]),
            
        'Z': ([0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.6],
              [0.0, 1.0, 0.0, 0.5, 1.0, 0.0, 1.0])
    }
    return position.get(letter, "Not a Letter")

def custom_word():
    goodRobot = 0
    while (goodRobot == 0):
        word=input("Enter word for robots to spell: ")
        word_parse = split(word)
        word_sum = 0

        for letter in word_parse:
            word_sum = word_sum + len(switch_position(letter)[0])
        
        if len(word_parse) == 0:
            print("Please Type a Different Word, You used too many Robots")

            
        if (word_sum <= 25 and word_sum !=0):
            goodRobot = 1
                
            if len(word_parse) == 1:
                for letter in word_parse:
                    x = switch_position(letter)[0]
                    y = switch_position(letter)[1]
                    for letter in x:
                        posX.append(letter + 2)

                    for letter in y:
                        posY.append(letter+2)
            
            if len(word_parse) == 2:
                offset = 0
                
                for letter in word_parse:
                    x = switch_position(letter)[0]
                    y = switch_position(letter)[1]
                    for letter in x:
                        posX.append(letter + 2 + offset)

                    for letter in y:
                        posY.append(letter+2)
                        
                    offset += 1
                    
            if len(word_parse) == 3:
                offset = 0
                
                for letter in word_parse:
                    x = switch_position(letter)[0]
                    y = switch_position(letter)[1]
                    for letter in x:
                        posX.append(letter + 1 + offset)

                    for letter in y:
                        posY.append(letter+2)
                        
                    offset += 1

            if len(word_parse) == 4:
                offset = 0
                
                for letter in word_parse:
                    x = switch_position(letter)[0]
                    y = switch_position(letter)[1]
                    for letter in x:
                        posX.append(letter + 1 + offset)

                    for letter in y:
                        posY.append(letter+2)
                        
                    offset += 1
                    
            if len(word_parse) == 5:
                offset = 0
                
                for letter in word_parse:
                    x = switch_position(letter)[0]
                    y = switch_position(letter)[1]
                    for letter in x:
                        posX.append(letter + offset)

                    for letter in y:
                        posY.append(letter+2)
                        
                    offset += 1
                    
    positionArray = [posX, posY]
    return positionArray
    
if __name__ == '__main__':
    start_time = time.time()
    custom_word()
    
