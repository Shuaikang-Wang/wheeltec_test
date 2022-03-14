#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from pywheeltecswarm import wheeltec

def callback(data):
    rospy.loginfo('The car is moving at linear velocity %f m/s.',
        data.twist.linear.x)
    rospy.loginfo('The car is moving at angular velocity %f rad/s.',
        data.twist.angular.z)


if __name__ == '__main__':
    rospy.init_node('wheeltec_listener', anonymous=False)

    rospy.Subscriber('/cmd_vel', TwistStamped, callback)

    rospy.spin()