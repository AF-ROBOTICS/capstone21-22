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
# | FILENAME      : usafalog.py
# | AUTHOR(S)     : C1C Anthony Tolbert
# | CREATED       : 11 Jan 2022
# | Last Update   : 06 Apr 2022
"""
This module uses a staggered-start (waterfall) technique as a method of collision avoidance to move robots to points.

The script uses other modules to assign a created list of robots (master class) destination positions via ROS. The
robots are then released one at a time. Each robot starts when the preceding robot reaches a user defined 'CLOSE'
distance to its destination point. Following completion, the script calls error_checking methods to collect and store
data.

Following completion, the script will enter a busy state and need to be ended with 'CTRL_C'. This may produce ROS errors
concerning messages being published to a closed topic, but these can usually be ignored without issue.

This script should be what is called to operate the robots in both simulation and real world using the command:
'rosrun capstone21-22 waterfall.py'

This script requires:
    * rospy
    * signal
    * PathBuild
    * DynaLet
    * error_checking
    * usafalog
    * master


This file contains 3 functions:
    * main : Create list of robots, assign destinations, waterfall robots to positions.
    * waterfall : Starts each robot when the one before it has reached its destination
    * ctrl_c_handler : Cleanly exits on keyboard interrupt
"""

import signal

import rospy

import DynaLet
import PathBuild
import error_checking
import master
import usafalog

logger = usafalog.CreateLogger(__name__)

NUM_BOTS = 25
BASENAME = 'usafabot'


def ctrl_c_handler(signum, frame):
    """Cleanly exits on keyboard interrupt"""
    print('\n')
    master.stop_pub(bots)
    rospy.signal_shutdown("CTRL C")
    logger.info("KILLED with CTRL_C")
    exit(1)


def waterfall():
    """Starts each robot when the one before it has reached its destination"""

    for bot in bots:
        if bot.dest_set and not bot.timeout:
            bot.start()
            while not bot.state == master.CLOSE or bot.state == master.DONE:
                pass
        else:
            logger.info(f"Skipping {bot.name}")
    logger.info("all bots complete")


bots = []


signal.signal(signal.SIGINT, ctrl_c_handler)


def main():
    """Create list of robots, assign destinations, waterfall robots to positions."""
    rospy.init_node('master', anonymous=True)
    # fill list of Master class bots
    for i in range(NUM_BOTS):
        bots.append(master.Master(BASENAME + str(i)))
    # Solicit pattern from user
    x_dyna, y_dyna, word = DynaLet.custom_word()
    if PathBuild.check_cache(word.upper()):
        x_dyna, y_dyna = PathBuild.check_cache(word.upper())
        logger.info("Using cached points")
    # Find all robot starting positions
    while not master.all_bots_found(bots): pass
    # Use DynaLet to assign robot destinations
    x_start, y_start = master.start_positions(bots)
    start_points, end_points = PathBuild.pack_to_points(x_dyna, y_dyna, x_start, y_start)
    x, y = PathBuild.build_path(start_points, end_points)

    if len(x):  # only move robots if assignment was successful
        PathBuild.add_to_cache(word.upper(), x, y)
        master.assign_bots(bots, x, y)
        waterfall()
        # Collect/store data
        error_checking.measure_error(bots)
        error_checking.breadcrumb_trail(bots)
        error_checking.plot_result(start_points, x, y)

    else:
        logger.warning("No bots assigned")
    logger.info("Waterfall Complete")


if __name__ == '__main__':
    main()
    rospy.spin()
