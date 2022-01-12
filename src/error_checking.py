import csv
from master import *
from datetime import datetime
from statistics import mean
from usafalog import *

# CSV filepath
path = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/"
# CSV filename for error measurement ex: 08Dec2022_14-40-43.csv
filename = datetime.now().strftime("%d%b%Y_%H-%M-%S") + ".csv"
outfile = path + filename
# CSV Headers
fields = ['bot', 'x_dest', 'x_avg_pos', 'y_dest', 'y_avg_pos', 'pos_err', 'time']


def measure_error(bots: list, num_samples=5, sample_period=10):
    logger.info(f"Collecting  f{num_samples} taken every  f{sample_period}  seconds")
    for Cycle in range(0, num_samples):
        for bot in bots:
            assert isinstance(bot, Master)
            # Add sample to array
            bot.x_avg.append(bot.curr_pos.position.x)
            bot.y_avg.append(bot.curr_pos.position.y)
        # Wait specified number of seconds before taking another sample
        time.sleep(sample_period)
        logger.debug(f"Finished sample:  f{Cycle + 1}")

    with open(outfile, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(fields)
        for bot in bots:
            bot.pos_err = ((mean(bot.x_avg) - bot.dest_pos.x) ** 2 + (
                    mean(bot.y_avg) - bot.dest_pos.y) ** 2) ** .5
            csvwriter.writerow(
                [bot.name, str(bot.dest_pos.x), (mean(bot.x_avg)), str(bot.dest_pos.y), str(mean(bot.y_avg)),
                 str(bot.pos_err), str(bot.time)])
    logger.info(f"CSV created with filename:  f{filename}")
