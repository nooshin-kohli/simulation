#! /usr/bin/env python3.6
# author: Noosin Kohli 
import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT
import time
import matplotlib.pyplot as plt
import sys

pub_hip = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)
pub_thigh = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
pub_calf = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)

rospy.init_node('command', anonymous=True)
tpre = time.time()

robot = ROBOT(np.zeros(3), np.zeros(3),
                  "/home/kamiab/catkin_ws/src/simulation/first_leg/scripts/leg_RBDL.urdf")  # TODO: give your own urdf_path

def callback(data):
    global tpre
    Qr  = np.array([0., 0., -np.pi/6]) 
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
    if not (Qr[1]==q_rbdl[1]):
        pub_thigh.publish(Qr[1])
    if not (Qr[2]==q_rbdl[2]):
        pub_calf.publish(Qr[2])
    if not (Qr[0]==q_rbdl[0]):
        pub_hip.publish(Qr[0])
    # if (rospy.Time.now().to_sec()>30):
    #     pub_slider.publish(-0.2)

    

    
    
    # pub_calf.publish(q_d[3])
    # pub_thigh.publish(q_d[2])
    # pub_hip.publish(q_d[1])
    # pub_slider.publish(0.4)


    # while ((rospy.get_time() - tpre) < 0.001): pass
    # tpre = rospy.get_time()


def main():
    # rospy.init_node('command', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()








if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
