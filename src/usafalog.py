import logging
import os

path = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/"
# path = "./"
# Setup logging
filename = path + "Waterfall.log"
fileformat = logging.Formatter("%(asctime)s-%(name)s-%(levelname)s:%(lineno)d:%(message)s")
consolefmt = logging.Formatter("%(levelname)s:%(message)s")

def CreateLogger(name, file_lvl=logging.DEBUG, console_lvl=logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)  # log everything

    console = logging.StreamHandler()
    console.setLevel(console_lvl)
    console.setFormatter(consolefmt)

    f_handler = logging.FileHandler(filename)
    f_handler.setLevel(file_lvl)
    f_handler.setFormatter(fileformat)

    logger.addHandler(console)
    logger.addHandler(f_handler)

    return logger
