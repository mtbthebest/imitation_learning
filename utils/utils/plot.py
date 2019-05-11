# coding=utf-8

from matplotlib import colors
import matplotlib.pyplot as plt


def plot_img_from_array(array):
    plt.imshow(array)
    plt.show()


def plot_curve(x_val, y_val, x_axis_title = 'X_axis', y_axis_title = 'Y_axis', color = 'red', smoothness = 1.0):
    fig = plt.figure()
    ax = plt.axes()


    # ax.plot(x_val[0::len(list(x_val)) / int(smoothness * len(list(x_val)))],
    #         y_val[0::len(list(x_val)) / int(smoothness * len(list(y_val)))], color = color)

    ax.plot(x_val,
            y_val, 'ro', color = color)
    plt.xlabel(x_axis_title)
    plt.ylabel(y_axis_title)
    plt.show()

# if __name__ == '__main__':
#     from file import Pd
#     import numpy as np
#
#     data = Pd(filename = '/home/mtb/Downloads/loss_AG_9.csv').read()
#     # print data
#     loss = data['Value'].values
#
#     print loss

    # step = np.arange(0,46000,46)
    #
    # plot_curve(step, loss, 'Step', 'Loss', smoothness = 0.75)
