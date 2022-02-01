#!/usr/bin/env python3
"""----------------------------------------------------------------------------------
Ground and Air Robot Teaming Capstone
Date: 11 Jan 2022
----------------------------------------------------------------------------------"""

import signal

import rospy

from error_checking import *
from master import *

logger = CreateLogger(__name__)
# TODO: what if you have more or less robots than needed?
NUM_BOTS = 25
BASENAME = 'usafabot'


def ctrl_c_handler(signum, frame):
    print('\n')
    stop_bots(bots)
    logger.info("KILLED with CTRL_C")
    exit(1)


bots = []
signal.signal(signal.SIGINT, ctrl_c_handler)
if __name__ == '__main__':
    rospy.init_node('master', anonymous=True)
    # fill list of Master class bots
    for i in range(0, NUM_BOTS):
        bots.append(Master(BASENAME + str(i)))
    stop_bots(bots)
    assign_bots(bots)
    for bot in bots:
        if not bot.timeout:
            bot.start()
            while not bot.state == CLOSE or bot.state == DONE:
                pass # TODO: How to not busy wait here
        else:
            logger.info(f"Skipping {bot.name}")

    logger.info("all bots complete")
    stop_bots(bots)
    measure_error(bots)
    exit(0)
