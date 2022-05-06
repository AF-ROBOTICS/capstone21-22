"""Used to compile the error data of a series of runs into one Point Comparison Plot"""
import visualizer

e = 0
tx = ''
path = '/home/dfec/Desktop/Combined/'
x_pp = []
y_pp = []
x_dd = []
y_dd = []
for i in range(9):
    file = 'Error Measurement' + str(i) + '.csv'
    x_d = []
    y_d = []
    x_p = []
    y_p = []
    x_d, y_d, x_p, y_p, e, t, tx = visualizer.read_error_file(path+file)
    x_pp.extend(x_p)
    y_pp.extend(y_p)
    x_dd.extend(x_d)
    y_dd.extend(y_d)
visualizer.points(x_pp, y_pp, x_dd, y_dd, tx, path)

