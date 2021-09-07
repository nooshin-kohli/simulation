#! /usr/bin/env python3.6
import rospy
from std_msgs.msg import Float64
import math

pub = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)


def callback(data):
    q = data
    pub.publish(q)

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     q_1 = 0.1  # this is in radians
    #     rospy.loginfo(q_1)
    #     pub.publish(q_1)
    #     rate.sleep()


def main():
    rospy.init_node('thigh_command', anonymous=True)
    rospy.Subscriber('/q_1', Float64, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
