#!/usr/bin/env python
import csv
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pywheeltecswarm import wheeltec as wt


def figure8():
    
    x = []
    y = []
    for i in range(0, 200):
        t = i/1000
        x.append(2*np.sin((10*t+1/360)*np.pi))
        y.append(2*np.sin(20*t*np.pi))
    pos = np.transpose([x,y])

    return pos


if __name__ == '__main__':

    time = wt.TimeHelper()
    wc = wt.Wheeltec(2)

    pos = figure8()
    data = pd.DataFrame(pos)
    data.to_csv('figure8.csv', index=False, header=False)

    wc.followPoints(pos,0.5)

    

    """
    t = range(0, np.shape(pos)[1])
    print(pos[:,0])

    plt.figure(figsize=(5, 5), dpi=100)
    plt.plot(pos[:,0], pos[:,1])

    plt.show()
    """
    

