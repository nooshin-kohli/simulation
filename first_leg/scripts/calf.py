#! /usr/bin/env python3.6
import rospy
from std_msgs.msg import Float64
import math



def calf():
    pub = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('calf_command', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        q_2 = -0.05  # this is in radians
        rospy.loginfo(q_2)
        pub.publish(q_2)
        rate.sleep()




if __name__ == '__main__':
    try:
        calf()
    except rospy.ROSInternalException:
        pass
