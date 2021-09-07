#! /usr/bin/env python3.6
import rospy
from std_msgs.msg import Float64
import math


def hip():
    pub = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('hip_command', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        q_0 = 0.3  # this is in radians
        rospy.loginfo(q_0)
        pub.publish(q_0)
        rate.sleep()








if __name__ == '__main__':
    try:
        hip()
    except rospy.ROSInternalException:
        pass
