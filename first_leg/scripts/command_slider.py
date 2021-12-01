#! /usr/bin/env python3.6
import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT
import time

pub_hip = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)
pub_thigh = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
pub_calf = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)
pub_slider = rospy.Publisher('/leg/jumper_position_controller/command', Float64, queue_size=10)
rospy.init_node('command', anonymous=True)
tpre = rospy.get_time()


def callback(data):
    global tpre
    q = data.position
    q = np.asarray(q)
    q_rbdl = np.zeros(4)
    q_rbdl[0] = q[2]
    q_rbdl[1] = q[1]
    q_rbdl[2] = q[3]
    q_rbdl[3] = q[0]
    qdot = data.velocity
    qdot = np.asarray(qdot)
    qdot_rbdl = np.zeros(4)
    qdot_rbdl[0] = qdot[2]
    qdot_rbdl[1] = qdot[1]
    qdot_rbdl[2] = qdot[3]
    qdot_rbdl[3] = qdot[0]
    robot = ROBOT(q_rbdl, qdot_rbdl,
                  "/home/kamiab/catkin_ws/src/simulation/first_leg/scripts/legRBDL.urdf")  # TODO: give your own urdf_path
    jc = robot.calcJc(q_rbdl)
    # print(q[0])
    pub_calf.publish(-0.1)
    pub_thigh.publish(0.1)
    pub_hip.publish(0.2)
    height = 0.4
    pub_slider.publish(height)

    while ((rospy.get_time() - tpre) < 0.001): pass
    tpre = rospy.get_time()


def main():
    # rospy.init_node('command', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass

