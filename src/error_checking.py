import csv
from statistics import mean
import os
import time
import master
import usafalog

logger = usafalog.CreateLogger(__name__)
logger.debug("Entering error")
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


def measure_error(bots: list, num_samples=5, sample_period=10):
    logger.info(f"Collecting  {num_samples} taken every  {sample_period}  seconds")
    for Cycle in range(0, num_samples):
        for bot in bots:
            assert isinstance(bot, master.Master)
            # Add sample to array
            bot.x_avg.append(bot.curr_pos.position.x)
            bot.y_avg.append(bot.curr_pos.position.y)
        # Wait specified number of seconds before taking another sample
        time.sleep(sample_period)
        logger.debug(f"Finished sample:  {Cycle + 1}")

    with open(outfile, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(fields)
        for bot in bots:
            bot.pos_err = ((mean(bot.x_avg) - bot.dest_pos.x) ** 2 + (
                    mean(bot.y_avg) - bot.dest_pos.y) ** 2) ** .5
            csvwriter.writerow(
                [bot.name, str(bot.dest_pos.x), (mean(bot.x_avg)), str(bot.dest_pos.y), str(mean(bot.y_avg)),
                 str(bot.pos_err), str(bot.time)])
    logger.info(f"CSV created with filename:  {filename}")
    # visualize(bots)


def breadcrumb_trail(bots):
    logger.info("Creating breadcrumb csvs")
    # Create folder for breadcrumb trail in specific run directory
    os.chdir(path)
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
