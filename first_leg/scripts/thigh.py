#! /usr/bin/env python3.6
import rospy
from std_msgs.msg import Float64
import math



def thigh():
    pub = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('thigh_command', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        q_1 = 0.1  # this is in radians
        rospy.loginfo(q_1)
        pub.publish(q_1)
        rate.sleep()


def calf():
    pub = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('command_joint', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        q_2 = 0.05  # this is in radians
        rospy.loginfo(q_2)
        pub.publish(q_2)
        rate.sleep()




if __name__ == '__main__':
    try:
        thigh()
    except rospy.ROSInternalException:
        pass
