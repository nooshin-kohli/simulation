#! /usr/bin/env python3.6
import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from VPend import VP


pub_1 = rospy.Publisher('/q_0', Float64, queue_size=10)
pub_2 = rospy.Publisher('/q_1',Float64,queue_size=10)
pub_3 = rospy.Publisher('/q_2',Float64,queue_size=10)
# q = []
# qdot = []
# jc = np.zeros((3, 3))


def callback(data):
    global q, qdot, jc
    q = data.position
    q = np.asarray(q)
    qdot = data.velocity
    qdot = np.asarray(qdot)
    r = VP(q, qdot)
    jc = r.calcJc(q)
    # print(type(jc))
    pub_1.publish(q[0])
    pub_2.publish(q[1])
    pub_3.publish(q[2])
    # print(str(jc))



def main():
    global q,qdot,jc
    rospy.init_node('jacobian', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()
    # pub = rospy.Publisher('/jc', Int32MultiArray, queue_size=10)
    # rospy.spin()
    # r = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     rospy.loginfo(jc)
    #     pub.publish(jc)
    #     r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass