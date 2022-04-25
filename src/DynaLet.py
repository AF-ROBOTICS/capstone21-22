#!/usr/bin/env python3
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
# | FILENAME      : DynaLet.py
# | AUTHOR(S)     : C1C Anthony Tolbert, C1C Matthew DeMaso
# | CREATED       : 09 Feb 2022
# | Last Update   : 14 Apr 2022
"""
This module allows the user to enter a custom phrase and get back a list of destinations for the robots to spell out
 that phrase using a dictionary of points for each individual letter

This script requires:
    * usafalog
    * statistics
    * PathBuild (debugging only)

This file contains 1 standalone functions and 1 main function for debugging:
    Function
    ---------
    custom_word : Word/phrase to be spelled with usafabots. Not used in most cases. When used, user input is bypassed
"""
from statistics import mean
import PathBuild
import usafalog
logger = usafalog.CreateLogger(__name__)
MAX_BOTS = 25
SCALE = 1
# Stores the positions of robots to make each letter in reference to the origin with a width of .6m and a height of 1m
# x and y coordinates are stored as parallel lists
letter_library = {
    'A': ([0.0, 0.6, 0.15, 0.45, 0.3],
          [0.0, 0.0, 0.50, 0.50, 1.0]),
    # TODO: B looks kinda weird
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
          [0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.5, 0.0]),

    'P': ([0.0, 0.0, 0.0, 0.3, 0.3, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 1.0]),

    'Q': ([0.00, 0.0, 0.0, 0.20, 0.2, 0.4, 0.40, 0.4, 0.6],
          [0.25, 0.7, 1.0, 0.25, 1.0, 1.0, 0.25, 0.7, 1.0]),
    # TODO: extra bots?
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

    'W': ([0.0, 0.0, 0.0, 0.3, 0.6, 0.6, 0.6],
          [0.0, 0.5, 1.0, 0.5, 0.0, 0.5, 1.0]),

    'X': ([0.0, 0.0, 0.3, 0.6, 0.6],
          [0.0, 1.0, 0.5, 0.0, 1.0]),

    'Y': ([0.0, 0.3, 0.3, 0.6],
          [1.0, 0.5, 0.0, 1.0]),

    'Z': ([0.0, 0.0, 0.3, 0.3, 0.3, 0.6, 0.6, 0.15, 0.45],
          [0.0, 1.0, 0.0, 0.5, 1.0, 0.0, 1.0, 0.75, 0.25])
}


def custom_word(usr_word=None):
    """
    Use a given word or solicit one from the user and return destination points to spell the word with usafabots

    Parameters
    ----------
    usr_word : str, optional
        Word/phrase to be spelled with usafabots. Not used in most cases. When used, user input is bypassed

    Returns
    --------
    x_destinations : list of floats
        1 of 2 parallel lists containing the x-coordinates of destination points
    y_destinations : list of floats
        1 of 2 parallel lists containing the y-coordinates of destination points
    usr_word : str
        word that is being spelled out by these points. (used for caching and logging especially when not passed as arg)
    """
    x_destinations = []
    y_destinations = []
    word_sum = 0  # Count of the number of robots needed to spell the particular word
    # Get user input. Only letters in dictionary allowed and cannot use more bots than there are
    while not 0 < word_sum <= MAX_BOTS:
        word_sum = 0  # Reset count at each iteration of the loop
        if usr_word is None:
            usr_word = input("Enter word for robots to spell: ")
            logger.debug(f"User entered \'{usr_word}\'")
        word_parsed = [char for char in usr_word.upper()]
        for letter in word_parsed:
            word_sum = word_sum + len(letter_library.get(letter, [[], []])[0])
            logger.debug(f"{usr_word} word takes {word_sum} bots to make")
            if word_sum > MAX_BOTS:
                logger.warning(f"'{usr_word}' take too many bots to spell ({word_sum})")
                if usr_word == "DFEC":  # Handle special case (DFEC is in cache)
                    word_sum = 25
                    break
                else:
                    usr_word = None

    # Separate each letter by 1 meter (box)
    for position, letter in enumerate(word_parsed):
        # Get the coordinates for each letter in phrase
        x_cords = letter_library.get(letter, [[], []])[0]
        y_cords = letter_library.get(letter, [[], []])[1]
        # Add spacing in the x direction in order the letters appear in the word
        for x_cord in x_cords:
            x_destinations.append((x_cord + position) * SCALE)
        # Add spacing in the y direction to center the robots along the vertical axis of the workspace
        for y_cord in y_cords:
            y_destinations.append(y_cord * SCALE)

    # Shift phrase to center of map (5.5m x 5m)
    x_center_offset = 5.5 / 2 - mean(x_destinations)
    y_center_offset = 5.0 / 2 - mean(y_destinations)
    for i in range(len(x_destinations)):
        x_destinations[i] += x_center_offset
        y_destinations[i] += y_center_offset

    return x_destinations, y_destinations, usr_word


if __name__ == '__main__':
    x_circ = [-0.002, 2.398, 5.998, 2.398, -0.002, 2.598, 5.998, 2.598, -0.002, 2.798, 5.998, 2.798, -0.002, 3.198,
              5.998, 3.198, -0.002, 3.398, 3.398, -0.002, 3.598, 3.598, -0.002, 3.798, -0.002]
    y_circ = [2.2, 6, 3.4, 0, 2.4, 6, 3.2, 0, 2.6, 6, 2.8, 0, 2.8, 6, 2.6, 0, 3.2, 6, 0, 3.4, 6, 0, 3.6, 6, 3.8]
    while True:
        x_dyna, y_dyna, word = custom_word()
        # if PathBuild.check_cache(word):
        #     x_dyna, y_dyna = PathBuild.check_cache(word)
        import matplotlib.pyplot as plt
        plt.plot(x_dyna, y_dyna, 'g8')
        plt.show()
        start_points, end_points = PathBuild.pack_to_points(x_dyna, y_dyna)
        x, y = PathBuild.build_path(start_points, end_points)
        # PathBuild.add_to_cache(word, x, y)
