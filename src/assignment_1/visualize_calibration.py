import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main(logfile):
    data = pd.read_csv(logfile, sep="\t", header=None)
    fig = plt.figure(1)

    ax1 = plt.subplot(211)
    line1 = plt.plot(data[0],data[2], label="Actual angle")
    line2 = plt.plot(data[0],data[1], label ="Reference angle")
    ax1.set_title('Left motor', fontsize=12)
    plt.xlabel('Time [ms]', fontsize=12)
    plt.ylabel('Angle [Radians]', fontsize=12)

    ax2 = plt.subplot(212)
    line1 = plt.plot(data[0],data[4], label="Actual angle")
    line2 = plt.plot(data[0],data[3], label ="Reference angle")
    ax2.set_title('Right motor', fontsize=12)
    plt.xlabel('Time [ms]', fontsize=12)
    plt.ylabel('Angle [Radians]', fontsize=12)

    plt.show()


if __name__ == '__main__':
    main("motor_position_1.log")
