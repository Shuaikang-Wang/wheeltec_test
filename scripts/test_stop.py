#!/usr/bin/env python

import rospy
import numpy as np
from pywheeltecswarm import wheeltec as wt

if __name__ == '__main__':
    #rospy.init_node('wheeltec01', anonymous=False)
    time = wt.TimeHelper()
    wc = wt.Wheeltec(2)

    # while not rospy.is_shutdown():
    wc.stop()
 



        
        

