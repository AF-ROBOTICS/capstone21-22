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
# | CONTACT       : (559) 326-4289, ajtolbert63@yahoo.com
# | CREATED       : 11 Jan 2022
# | Last Update   : 06 Apr 2022
"""
This module allows the user to take advantage of the native logging module, while abstracting away complex and
repetitive parameters. The system also makes it easier to use the same logging file for multiple modules in the Python
space.

The system is set up primarily for the Waterfall implementation. The logger, when called for the first time, creates a
new folder to store that runs files including: the log file, the BreadCrumb .csv s, and any created images.

This script requires:
    * logging
    * os
    * datetime

This module is assumed to be imported. When imported, the CreateLogger() method is called which allows the created log
file to differentiate which module is making the logging entry.

This file contains one function:
    * CreateLogger - returns a logging.Logger object to be referenced by the calling module for logging
"""

import logging
import os
from datetime import datetime

# Directory name for data collection ex: 08Dec2022_14-40-43.csv
directory_name = datetime.now().strftime("%d%b%Y_%H-%M-%S") + '/'
path = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/" + directory_name
# Create and operate in the new directory
os.mkdir(path)
os.chdir(path)

# Setup logging
filename = path + "Waterfall.log"
# Line format in the file. ex: 2022-04-04 10:05:28,446-__main__-INFO:61:Waterfall Complete
fileformat = logging.Formatter("%(asctime)s-%(name)s-%(levelname)s:%(lineno)d:%(message)s")
# Line format to the console. ex: INFO:Using cached points
consolefmt = logging.Formatter("%(levelname)s:%(message)s")

f_handler = logging.FileHandler(filename, mode='w')


def CreateLogger(name, file_lvl=logging.DEBUG, console_lvl=logging.INFO):
    """Creates a logging.Logger instance to consolidate logging in both files and to the console

    Sets common parameters across all logging instances

    Parameters
    ----------
    name : str
        The __name__ of the calling module
    file_lvl : logging.[level], optional (default: logging.DEBUG)
        The severity level to be included in the created .log file
    console_lvl : logging.[level], optional (default: logging.INFO)
        The severity level to be included in logging to the console

    Returns
    -------
    logger
        a logging.Logger instance
    """

    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)  # log everything

    console = logging.StreamHandler()
    console.setLevel(console_lvl)
    console.setFormatter(consolefmt)

    f_handler.setLevel(file_lvl)
    f_handler.setFormatter(fileformat)

    logger.addHandler(console)
    logger.addHandler(f_handler)

    return logger
