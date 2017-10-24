import math
import pandas as pd
import numpy as np


def main(logfile):
    data = pd.read_csv(logfile, sep=" ", header=None)
    print ("Mean of X: {}".format(np.mean(data[0])))
    print ("Mean of Y: {}".format(np.mean(data[1])))
    data = np.vstack((data[0],data[1]))
    print("Covariance:")
    print(np.cov(data))


if __name__ == '__main__':
    main("logs/error.log")
