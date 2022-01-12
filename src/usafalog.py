import logging
import os

path = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/"

# Setup logging
filename = path + "Waterfall.log"
fileformat = logging.Formatter("%(asctime)s-%(name)-%(levelname)s:%(lineno)s:%(message)s")
consolefmt = logging.Formatter("%(levelname)s:%(message)s")
logger = logging.getLogger(__name__)
console = logging.StreamHandler()
console.setLevel(logging.WARNING)
console.setFormatter(consolefmt)
f_handler = logging.FileHandler(filename)
f_handler.setLevel(logging.DEBUG)
f_handler.setFormatter(fileformat)
logger.addHandler(console)
logger.addHandler(f_handler)
