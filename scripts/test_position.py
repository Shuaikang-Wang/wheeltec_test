#!/usr/bin/env python

import rospy
import numpy as np
from pywheeltecswarm import wheeltec as wt

if __name__ == '__main__':

    time = wt.TimeHelper()
    wc = wt.Wheeltec(2)

    while not rospy.is_shutdown():
        wc.cmdVelocity([0,0], 0.5)
        print(wc.position())
        time.sleep(0.1)

    wc.cmdVelocity([0,0], 0.0)
        
        

