#!/usr/bin/env python

import rospy
import numpy as np
from pywheeltecswarm import wheeltec as wt

if __name__ == '__main__':
    rospy.init_node('wheeltec01', anonymous=False)
    time = wt.TimeHelper()
    wc = wt.Wheeltec(2)
    
    data = np.loadtxt("waypoints.csv", delimiter=',')
    
    wc.followPoints(data,0.01)


    
    





        
        

