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
# | FILENAME      : visualizer.py
# | AUTHOR(S)     : C1C Anthony Tolbert
# | CONTACT       : (559) 326-4289, ajtolbert63@yahoo.com
# | CREATED       : 22 Feb 2022
# | Last Update   : 14 Apr 2022
"""
This module serves to create visual plots using matplotlib.pyplot to interpret and report data collected during a
 usafabot run

This module is needed and should run separately from any ros items due to a sporadic error with the interaction of
 tkinter (used by pyplot) and ROS. This module also assumes the same file structure that is created in the
 error_measurement module. It only needs to be pointed to the folder with that data that is desired to be visualized.

This script requires:
    * csv
    * os
    * statistics
    * tkinter
    * matplotlib.pyplot

This file contains 3 standalone functions and 1 main function:
    Functions
    ---------
    * points : Creates a point comparison plot for robot actual positions versus destination positions
    * paths : Plots breadcrumb trails from created .csvs
    * read_error_file : Read and interpret csv data created in the error_measurement module
"""
import csv
import os
from statistics import mean, median
from tkinter import *
from tkinter import filedialog

import matplotlib.pyplot as plt


def points(x_pos, y_pos, x_dest, y_dest, text, path):
    """
    Creates a point comparison plot for robot actual positions versus destination positions

    Parameters
    ----------
    x_pos : list
        list of x coordinates for actual robot positions
    y_pos : list
        list of x coordinates for actual robot positions
    x_dest : list
        list of x coordinates for desired robot positions
    y_dest : list
        list of x coordinates for desired robot positions
    text : str
        text to be written on plots. Contains mean position error and mean time
    path : str
        file path to the directory where the plot should be stored
    """
    # Desired points are green dots. Actual positions are blue stars
    for i in range(len(x_pos)):
        plt.plot(x_dest[i], y_dest[i], 'g8', x_pos[i], y_pos[i], 'b*')
    plt.axis([0, 6, 0, 6])
    plt.xlabel("East-West Axis of Robot Workspace (m)")
    plt.ylabel("North-South Axis of Robot Workspace (m)")
    plt.title(f"Point Comparison")
    plt.grid()
    plt.text(.5, .5, text)
    plt.legend(["Goal Positions", "Actual Positions"])
    # plt.show()
    plt.savefig(path + '/Point Comparison' + ".png")


def paths(path):
    """
    Plots breadcrumb trails from created .csvs

    Parameter
    ---------
    path : str
        file path to the directory where the plot should be stored
    """
    # Give each bot a unique marker/color combination
    colors = ['r', 'g', 'b', 'm', 'k']
    markers = ['.', '1', '*', '^']
    i = 0
    # Assumes that the current folder has the breadcrumb files in a separate directory
    try:
        os.chdir(path + '/BreadCrumbs')
        # Read files for a maximum of 25 robots and discard robots that didn't move.
        all_filenames = ['usafabot' + str(i) + '.csv' for i in range(25)]
        for filename in all_filenames:
            current_points = []
            try:
                with open(filename) as current_file:
                    csvreader = csv.reader(current_file)
                    # Get rid of headers and blanks
                    csvreader.__next__()
                    csvreader.__next__()
                    csvreader.__next__()
                    # Each row contains an (x,y) breadcrumb
                    for row in csvreader:
                        # print(row)
                        current_points.append(row)
                    x = []
                    y = []
                    # Separate into x and y lists to plot
                    for point in current_points:
                        x.append(float(point[0]))
                        y.append(float(point[1]))
                    # Plot each trail using a different line/marker combination
                    plt.plot(x, y, linestyle='--', marker=markers[i % len(markers)], color=colors[i % len(colors)])
                    i += 1
            except OSError:
                print(f"Unable to open {filename}")
        plt.axis([0, 6, 0, 6])
        plt.xlabel("East-West Axis of Robot Workspace (m)")
        plt.ylabel("North-South Axis of Robot Workspace (m)")
        plt.title(f"Breadcumb Trails")
        plt.grid()
        # plt.show()
        plt.savefig(path + '/Breadcrumb Trail' + ".png")
    except OSError:
        print(f"No folder '/BreadCrumbs' in {path}")


def read_error_file(csv_name):
    """
    Read and interpret csv data created in the error_measurement module

    Parameter
    ---------
    csv_name : str
        filename of the csv with the desired data created by the error_measurement module
    """
    # Empty lists to store read data
    bot = []
    x_dest = []
    y_dest = []
    x_pos = []
    y_pos = []
    error = []
    time = []
    text = ""

    try:
        with open(csv_name, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            # Skip column headers
            csvreader.__next__()
            for row in csvreader:
                if float(row[6]) != 0:  # Only read data for robots that actually moved (time > 0)
                    # print(row)
                    bot.append(row[0])  # Name
                    x_dest.append(float(row[1]))
                    x_pos.append(float(row[2]))
                    y_dest.append(float(row[3]))
                    y_pos.append(float(row[4]))
                    error.append(float(row[5]))
                    if float(row[6]) < 1800:
                        time.append(float(row[6]))
                    else:
                        print(f"WARNING: {row[0]} time rejected for being too high ({row[6]})")
            text = f"Mean error (cm): {round(mean(error) * 100, 2)}\nMean time (s): {round(mean(time), 2)}\n" \
                   f"Median time (s): {round(median(time), 2)}"
            print(text)
    except OSError:
        print(f"No file found {csv_name}")
    return x_dest, y_dest, x_pos, y_pos, error, time, text


def main():
    # GUI setup
    root = Tk()
    root.withdraw()
    # Get path from user, assuming setup by waterfall run and cwd is capstone21-22/src
    path = filedialog.askdirectory(initialdir='../measurement_files')
    try:
        os.chdir(path)
        x_dest, y_dest, x_pos, y_pos, error, time, text = read_error_file(path + '/Error Measurement.csv')
        if x_dest != []:
            points(x_pos, y_pos, x_dest, y_dest, text, path)
            plt.close('all')  # In case plots are shown and not saved
        else:
            print(f"WARNING: No data read from {path + '/Error Measurement.csv'}")
        paths(path)
    except TypeError as err:
        print("Dialog closed without selecting a folder. No action taken!")


if __name__ == "__main__":
    main()
