#!/usr/bin/env python

import rospy
import numpy as np
from pywheeltecswarm import wheeltec as wt

if __name__ == '__main__':

    time = wt.TimeHelper()
    wc = wt.Wheeltec(2)

    pos0 = np.array([0.0,0.0])
    pos1 = np.array([1.0,0.0])
    pos2 = np.array([1.0,1.0])
    pos3 = np.array([0.0,1.0])
    yaw = np.pi/2

    # while not rospy.is_shutdown():
    time.sleep(1)
    '''
    wc.goTo(pos0, 0.0, 3, relative=False)
    print("Moving to (%f,%f)" %(pos0[0],pos0[1]))
    '''
    print("Moving to (%f,%f)" %(pos0[0],pos0[1]))
    wc.goTo(pos0, 0.0, 10, relative=False)
    print("Moving to (%f,%f)" %(pos1[0],pos1[1]))
    wc.goTo(pos1, np.pi/2, 10, relative=False)
    print("Moving to (%f,%f)" %(pos2[0],pos2[1]))
    wc.goTo(pos2, np.pi, 10, relative=False)
    print("Moving to (%f,%f)" %(pos3[0],pos3[1]))
    wc.goTo(pos3, np.pi*3/2, 10, relative=False)
    print("Moving to (%f,%f)" %(pos0[0],pos0[1]))
    wc.goTo(pos0, 0.0, 10, relative=False)

    for i in range(5):
        print(wc.position())
        time.sleep(0.1)

    wc.stop()
    
    """
    for i in range(100):
        if not time.isShutdown():
            wc.goTo(pos2, 0.0)
            print("Moving to (%f,%f)" %(pos2[0],pos2[1]))
            time.sleep(0.1)
        

    for i in range(100):
        if not time.isShutdown():
            wc.goTo(pos3, 0.0)
            print("Moving to (%f,%f)" %(pos3[0],pos3[1]))
            time.sleep(0.1)
    """


        
        

