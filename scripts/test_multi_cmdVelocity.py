#!/usr/bin/env python

from numpy.core.numeric import zeros_like
import rospy
import numpy as np
from pywheeltecswarm import wheeltec as wt

if __name__ == '__main__':
    
    time = wt.TimeHelper()

    wc1 = wt.Wheeltec(1)
    wc2 = wt.Wheeltec(2)


    velx = np.array([0.5,0.0,0.0])
    vely = np.array([0.0,0.5,0.0])
    zeros = np.array([0.0,0.0,0.0])
    yawRate = 0.8

    rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():

    while not time.isShutdown():
        for i in range(10):
            if not time.isShutdown():
                wc1.cmdVelocityWorld(velx, 0.0)
                print("Car 1 is moving in the x direction at rate " + str(velx[0]) + " m/s")
                
                wc2.cmdVelocityWorld(vely, 0.0)
                print("Car 2 is moving in the y direction at rate " + str(vely[1]) + " m/s")
                
                rate.sleep()

        for i in range(10):
            if not time.isShutdown():
                wc1.cmdVelocityWorld(zeros, yawRate)
                print("Car 1 is rotating at rate " + str(yawRate) + " rad/s")

                rate.sleep()
        
        

