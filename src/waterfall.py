#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground and Air Robot Teaming Capstone
Date: 11 Jan 2022
----------------------------------------------------------------------------------"""

import rospy
import signal
import PathBuild
import DynaLet
import time
import error_checking
import usafalog
import master

logger = usafalog.CreateLogger(__name__)

NUM_BOTS = 25
BASENAME = 'usafabot'


def ctrl_c_handler(signum, frame):
    print('\n')
    master.stop_bots(bots)
    logger.info("KILLED with CTRL_C")
    exit(1)


bots = []
signal.signal(signal.SIGINT, ctrl_c_handler)
if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # fill list of Master class bots
    for i in range(0, NUM_BOTS):
        bots.append(master.Master(BASENAME + str(i)))
    init_time = time.perf_counter()
    # stop_bots(bots)
    x_dyna, y_dyna, word = DynaLet.custom_word()
    if PathBuild.check_cache(word):
        x_dyna, y_dyna = PathBuild.check_cache(word)
    start_points, end_points = PathBuild.pack_to_points(x_dyna, y_dyna)
    x, y = PathBuild.buildPath(start_points, end_points)
    PathBuild.add_to_cache(word, x, y)
    master.assign_bots(bots, x, y)
    while time.perf_counter() - init_time < 15: pass
    for bot in bots:
        if bot.dest_set and not bot.timeout:
            bot.start()
            while not bot.state == master.CLOSE or bot.state == master.DONE:
                pass  # TODO: How to not busy wait here
        else:
            logger.info(f"Skipping {bot.name}")
    logger.info("all bots complete")
    # stop_bots(bots)
    error_checking.measure_error(bots)
    rospy.spin()
