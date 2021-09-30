#! /usr/bin/env python3.6
import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT
import time

# pub_1 = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)
# pub_2 = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
# pub_3 = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)

rospy.init_node('command', anonymous=True)
tpre = rospy.get_time()

def callback(data):
    global tpre
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
    robot = ROBOT(q_rbdl, qdot_rbdl, "/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")  # TODO: give your own urdf_path
    jc = robot.calcJc(q)
    print(robot.pose_end(q_rbdl))
    # print(q)
    # pub_1.publish(q[0]+0.1)
    # pub_2.publish(q[1]+0.1)
    # pub_3.publish(1.0)

    while ((rospy.get_time() - tpre)< 0.001):pass
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
