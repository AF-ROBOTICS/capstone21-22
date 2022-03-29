import csv
import os
from statistics import mean
from tkinter import *
from tkinter import filedialog

import matplotlib.pyplot as plt


def points(x_pos, y_pos, x_dest, y_dest, text, path):
    for i in range(len(x_pos)):
        plt.plot(x_dest[i], y_dest[i], 'g8', x_pos[i], y_pos[i], 'b*')
    plt.axis([0, 6, 0, 6])
    plt.xlabel("East-West Axis of Robot Workspace (m)")
    plt.ylabel("North-South Axis of Robot Workspace (m)")
    plt.text(.5, .5, text)
    plt.legend(["Goal Positions", "Actual Positions"])
    # plt.show()
    plt.savefig(path + '/Point Comparison' + ".png")


def paths(path):
    colors = ['r', 'g', 'b', 'm', 'k']
    markers = ['.', '1', '*', '^']
    i = 0
    os.chdir(path + '/BreadCrumbs')
    all_filenames = ['usafabot' + str(i) + '.csv' for i in range(25)]
    for filename in all_filenames:
        current_points = []
        with open(filename) as current_file:
            csvreader = csv.reader(current_file)
            # Get rid of headers and blanks
            csvreader.__next__()
            csvreader.__next__()
            csvreader.__next__()
            # Each row contains an (x,y) breadcrumb
            for row in csvreader:
                # print(row)
                current_points.append(row)
            x = []
            y = []
            # Separate into x and y lists to plot
            for point in current_points:
                x.append(float(point[0]))
                y.append(float(point[1]))
            # Plot each trail using a different line/marker combination
            plt.plot(x, y, linestyle='--', marker=markers[i % len(markers)], color=colors[i % len(colors)])
            i += 1
    plt.axis([0, 6, 0, 6])
    plt.xlabel("East-West Axis of Robot Workspace (m)")
    plt.ylabel("North-South Axis of Robot Workspace (m)")
    # plt.show()
    plt.savefig(path + '/Breadcrumb Trail' + ".png")


def read_error_file(csv_name):
    bot = []
    x_dest = []
    y_dest = []
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
        text = f"Mean error (cm): {round(mean(error) * 100, 2)}\nMean time (s): {round(mean(time), 2)}"
        print(text)
    return x_dest, y_dest, x_pos, y_pos, error, time, text


if __name__ == "__main__":
    root = Tk()
    root.withdraw()
    # Get path from user, assuming setup by waterfall run
    path = filedialog.askdirectory(initialdir='../measurement_files')
    os.chdir(path)
    x_dest, y_dest, x_pos, y_pos, error, time, text = read_error_file(path + '/Error Measurement.csv')
    points(x_pos, y_pos, x_dest, y_dest, text, path)
    plt.close('all')
    paths(path)
