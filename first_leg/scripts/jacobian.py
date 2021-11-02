#! /usr/bin/env python3.6

'''
   Autor : Nooshin Kohli
   
'''
import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT
import matplotlib.pyplot as plt


# pub_1 = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)
# pub_2 = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
# pub_3 = rospy.Publisher('/leg/calf_joint_position_controller/command',Float64,queue_size=10)
# q = []
# qdot = []
# jc = np.zeros((3, 3))
pub = rospy.Publisher('/end_jacobian', MultiArrayDimension, queue_size=10)
i = 0
l_one = []
def callback(data):
    global q, qdot, jc,i
    q = data.position
    q = np.asarray(q)
    q_rbdl = np.zeros(3)
    q_rbdl[0] = q[1]
    q_rbdl[1] = q[2]
    q_rbdl[2] = q[0]
    qdot = data.velocity
    qdot = np.asarray(qdot)
    qdot_rbdl = np.zeros(3)
    qdot_rbdl[0] = qdot[1]
    qdot_rbdl[1] = qdot[2]
    qdot_rbdl[2] = qdot[0]
    leg = ROBOT(q, qdot, "/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")
    jc = leg.calcJc(q_rbdl)
    i = i +1
    l_one.append(i)
    #print (jc)
    # print(type(jc))
    # pub_1.publish(q[0])
    # pub_2.publish(q[1])
    # pub_3.publish(q[2])
    # print(str(jc))



def main():
    rospy.init_node('jacobian', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()
    while rospy.is_shutdown():
        print("ROS is SHUTDOWN!!")
        plt.plot(l_one,'-')
        plt.show()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
