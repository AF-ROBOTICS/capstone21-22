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
# | FILENAME      : error_checking.py
# | AUTHOR(S)     : C1C Anthony Tolbert
# | CONTACT       : (559) 326-4289, ajtolbert63@yahoo.com
# | CREATED       : 11 Jan 2022
# | Last Update   : 18 Apr 2022
"""
This module provides functionality for measurement, recording, and calculation of data collected during a run.

While designed with the waterfall implementation in mind these functions should be adaptable to all future
 implementations

This script requires:
    * csv
    * os
    * time
    * usafalog
    * statistics
    * matplotlib.pyplot
    * master

This file contains 3 standalone functions:
    Functions
    ---------
    * measure_error : Record data collection fields in Master class to csv file
    * breadcrumb_trail : Create csv file for breadcrumb trail data points
    * plot_result : Plot the intended paths for each robot and save as png file
"""
import csv
import os
import time
from statistics import mean
from matplotlib import pyplot as plt
import master
import usafalog

logger = usafalog.CreateLogger(__name__)
logger.debug("Entering error_checking")
# CSV filepath
path = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/"
os.chdir(path)
# Assume operating out of the most recently created directory
all_sdirs = [d for d in os.listdir('.') if os.path.isdir(d)]
current_dir = max(all_sdirs, key=os.path.getmtime)
# Update path to be in the run's specific directory
path = path + current_dir + '/'
filename = 'Error Measurement'
outfile = path + filename + ".csv"
# CSV Headers
fields = ['bot', 'x_dest', 'x_avg_pos', 'y_dest', 'y_avg_pos', 'pos_err', 'time']


def measure_error(bots: list, num_samples=2, sample_period=10):
    """
    Record data collection fields in Master class to csv file

    Parameters
    ----------
    bots : list
        list of Master class representing robots
    num_samples : int, optional
        used for averaging the final position measurements. This is mostly a holdover from when the bots moved after
         reaching their final destination
    sample_period : int, optional
        time to wait (in seconds) between samples collected for averaging final position measurements. This is mostly a
         holdover from when the bots moved after reaching their final destination
    """
    logger.info(f"Collecting  {num_samples} taken every  {sample_period}  seconds")
    for cycle in range(num_samples):
        for bot in bots:
            assert isinstance(bot, master.Master)
            # Add sample to array
            bot.x_avg.append(bot.curr_pos.position.x)
            bot.y_avg.append(bot.curr_pos.position.y)
        # Wait specified number of seconds before taking another sample
        time.sleep(sample_period)
        logger.debug(f"Finished sample:  {cycle + 1}")

    with open(outfile, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write headers
        csvwriter.writerow(fields)
        for bot in bots:
            # Calculate radial from destination
            bot.pos_err = ((mean(bot.x_avg) - bot.dest_pos.x) ** 2 + (
                    mean(bot.y_avg) - bot.dest_pos.y) ** 2) ** .5
            # Write a new row for each robot
            csvwriter.writerow(
                [bot.name, str(bot.dest_pos.x), (mean(bot.x_avg)), str(bot.dest_pos.y), str(mean(bot.y_avg)),
                 str(bot.pos_err), str(bot.time)])
    logger.info(f"CSV created with filename:  {filename}")


def breadcrumb_trail(bots: list):
    """
    Create csv file for breadcrumb trail data points

    Parameters
    ----------
    bots : list
        list of Master class representing robots
    """
    logger.info("Creating breadcrumb csvs")
    # Create folder for breadcrumb trail in specific run directory
    os.chdir(path)
    # Store all csv in the same folder. Separate csv for each bot
    os.mkdir('BreadCrumbs')
    os.chdir('./BreadCrumbs')
    # save each bot's breadcrumb trail in a csv
    for bot in bots:
        with open(f"{bot.name}" + ".csv", "w") as BCfile:
            csvwriter = csv.writer(BCfile)
            logger.debug(f"Writing csv for {bot.name}")
            csvwriter.writerow(['x', 'y'])
            for point in bot.breadcrumbs:
                csvwriter.writerow(point)


def plot_result(starts, xpoints, ypoints):
    """
    Plot the intended paths for each robot and save as png file

    Parameters
    ----------
    starts : list of points
        list of the originating points for the robots. Stored in order 0->24
    xpoints : list of floats
        list of the x coordinates of the destination positions for each robot stored in order 0->24
    ypoints : list of floats
        list of the y coordinates of the destination positions for each robot stored in order 0->24
    """
    for i in range(0, len(xpoints)):
        plt.plot([starts[i].x, xpoints[i]], [starts[i].y, ypoints[i]])
        plt.plot(xpoints, ypoints, 'bo')
    plt.axis([0, 6, 0, 6])
    plt.xlabel("East-West Axis of Robot Workspace (m)")
    plt.ylabel("North-South Axis of Robot Workspace (m)")
    plt.title(f"Planned paths")
    plt.grid()
    plt.savefig(path + 'Planned Paths' + ".png")
    # plt.show()
