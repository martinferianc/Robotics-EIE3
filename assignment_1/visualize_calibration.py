import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.patches as mpatches


def main(logfile):
    logfile_path = './logs/' + logfile
    data = pd.read_csv(logfile_path, sep="\t", header=None)
    fig = plt.figure(1)

    # set figure title
    plt.suptitle(logfile)

    ax1 = plt.subplot(211)
    ax1_line1 = plt.plot(data[0],data[2], "g", label="Actual angle")
    ax1_line2 = plt.plot(data[0],data[1], "b", label ="Reference angle")
    ax1.set_title('Left motor', fontsize=12)
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('Angle [Radians]', fontsize=12)

    # create legends
    green_patch = mpatches.Patch(color='green', label='Actual angle')
    blue_patch = mpatches.Patch(color='blue', label='Reference angle')

    # place legends on the bottom right
    plt.legend(handles=[green_patch, blue_patch], loc=4)

    ax2 = plt.subplot(212)
    ax2_line1 = plt.plot(data[0],data[4], "g", label="Actual angle")
    ax2_line2 = plt.plot(data[0],data[3], "b", label ="Reference angle")
    ax2.set_title('Right motor', fontsize=12)
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('Angle [Radians]', fontsize=12)

    # place legends on the bottom right
    plt.legend(handles=[green_patch, blue_patch], loc=4)

    # prevent overlapping
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # save as png
    graph_path = './graphs/' + logfile + '.png'
    plt.savefig(graph_path, dpi=140)

    # plt.show()


if __name__ == '__main__':
    #main("motor_position_1_60.0.log")
    #main("motor_position_1_180.0.log")
    main("motor_position_1_60.0.log")
