import math
import pandas as pd
import numpy as np


def main(logfile):
    data = pd.read_csv(logfile, sep=" ", header=None)
    data = np.vstack((data[0],data[1]))
    print(np.cov(data))


if __name__ == '__main__':
    main("logs/error.log")
