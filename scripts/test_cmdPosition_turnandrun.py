#!/usr/bin/env python

import rospy
import numpy as np
from pywheeltecswarm import wheeltec as wt

if __name__ == '__main__':

    time = wt.TimeHelper()
    wc = wt.Wheeltec(2)

    time.sleep(1)
    
    pos1 = [1.0,0.0]
    pos2 = [1.0,1.0]

    wc.cmdPosition_turnandrun(pos1)
    print("Moving to (%f,%f)" %(pos1[0],pos1[1]))

    wc.cmdPosition_turnandrun(pos2)
    print("Moving to (%f,%f)" %(pos2[0],pos2[1]))



        
        

