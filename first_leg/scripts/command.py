#! /usr/bin/env python3.6
import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT

pub_1 = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)
pub_2 = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
pub_3 = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)

# tpre = rospy.get_time()

def callback(data):
    # global tpre
    q = data.position
    q = np.asarray(q)
    qdot = data.velocity
    qdot = np.asarray(qdot)
    robot = ROBOT(q, qdot, "/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")  # TODO: give your own urdf_path
    jc = robot.calcJc(q)
    # print(q[0])
    pub_1.publish(q[0]+0.1)
    pub_2.publish(q[1]+0.1)
    pub_3.publish(q[2]+0.1)

    # while ((rospy.get_time() - tpre)< 0.001):pass
    # tpre = rospy.get_time()



def main():
    rospy.init_node('command', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
