#!/usr/bin/env python

import rospy
import numpy as np
from pywheeltecswarm import wheeltec

if __name__ == '__main__':
    rospy.init_node('wheeltec01', anonymous=False)
    try:
        # for i=1:10
        wc = wheeltec.Wheeltec(1)
        vel = np.array([0.05, 0, 0])
        yawRate = 0.1
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            wc.cmdVelocityWorld(vel, yawRate)
            rospy.loginfo("The car is moving …")
            rate.sleep()
            # print("The car is moving …")
            # talker()

    except rospy.ROSInterruptException:
        pass
