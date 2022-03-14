#!/usr/bin/env python

import rospy
import numpy as np
from pywheeltecswarm import wheeltec as wt

if __name__ == '__main__':
    #rospy.init_node('wheeltec01', anonymous=False)
    time = wt.TimeHelper()
    wc = wt.Wheeltec(2)

    velx = [0.5,0.0]
    vely = [0.0,0.5]
    zeros = [0.0,0.0]
    yawRate = 0.8

    # while not rospy.is_shutdown():
    rate = rospy.Rate(10) # 发布消息的频率 10Hz

    while not time.isShutdown():
        # for i in range(10):
        #     if not time.isShutdown():
        #         wc.cmdVelocity(velx, 0.0)
        #         print("The car is moving in the x direction at rate " + str(velx[0]) + " m/s")
        #         rate.sleep()                

        # for i in range(10):
        #     if not time.isShutdown():
        #         wc.cmdVelocity(vely, 0.0)
        #         print("The car is moving in the y direction at rate " + str(vely[1]) + " m/s")
        #         #rospy.loginfo("The car is moving on y at rate " + vely[2] )
        #         rate.sleep()            

        for i in range(10):
            if not time.isShutdown():
                wc.cmdVelocity(zeros, yawRate)
                print("The car is rotating at rate " + str(yawRate) + " rad/s")
                rate.sleep()   
        
        for i in range(10):
            if not time.isShutdown():
                wc.cmdVelocity(zeros, -yawRate)
                print("The car is rotating at rate " + str(-yawRate) + " rad/s")
                rate.sleep()   

        wc.stop()


        
        

