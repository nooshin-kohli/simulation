#! /usr/bin/env python3.6
import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT


# pub_1 = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)
# pub_2 = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
# pub_3 = rospy.Publisher('/leg/calf_joint_position_controller/command',Float64,queue_size=10)
# q = []
# qdot = []
# jc = np.zeros((3, 3))
pub = rospy.Publisher('/end_jacobian', Int32MultiArray, queue_size=10)

def callback(data):
    global q, qdot, jc
    q = data.position
    q = np.asarray(q)
    qdot = data.velocity
    qdot = np.asarray(qdot)
    r = ROBOT(q, qdot, "/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")
    jc = r.calcJc(q)
    # tuple(map(tuple,jc))
    # pub.publish(jc)

    # print(type(jc))
    # pub_1.publish(q[0])
    # pub_2.publish(q[1])
    # pub_3.publish(q[2])
    print(str(jc))



def main():
    rospy.init_node('jacobian', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
