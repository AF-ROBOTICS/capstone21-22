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
fileformat = logging.Formatter("%(asctime)s-%(name)s-%(levelname)s:%(lineno)d:%(message)s")
consolefmt = logging.Formatter("%(levelname)s:%(message)s")

f_handler = logging.FileHandler(filename, mode='w')


def CreateLogger(name, file_lvl=logging.DEBUG, console_lvl=logging.INFO):
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
