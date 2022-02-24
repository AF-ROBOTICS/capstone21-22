import csv
from datetime import datetime
import os
import matplotlib.pyplot as plt
from statistics import mean
path = "/home/" + os.getlogin() + "/robotics_ws/src/capstone21-22/measurement_files/"
filename = datetime.now().strftime("%d%b%Y_%H-%M-%S")
csv_name = path + "23Feb2022_15-49-02.csv"

def visualize(x_pos, y_pos, x_dest, y_dest):
    for i in range(len(x_pos)):
        plt.plot(x_dest[i], y_dest[i], 'g8', x_pos[i], y_pos[i], 'b*')
    plt.axis([0, 6, 0, 6])
    # plt.show()
    plt.savefig(path + filename + ".png")


if __name__ == "__main__":
    bot = []
    x_dest =[]
    y_dest =[]
    x_pos = []
    y_pos = []
    error = []
    time = []
    with open(csv_name, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        csvreader.__next__()
        for row in csvreader:
            if float(row[6]) != 0:
                print(row)
                bot.append(row[0])
                x_dest.append(float(row[1]))
                x_pos.append(float(row[2]))
                y_dest.append(float(row[3]))
                y_pos.append(float(row[4]))
                error.append(float(row[5]))
                time.append(float(row[6]))
        print(f"Mean error (cm): {round(mean(error)*100,4)}")
        print(f"Mean time (s): {mean(time)}")

    visualize(x_pos, y_pos, x_dest, y_dest)

