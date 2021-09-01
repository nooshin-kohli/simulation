#! /usr/bin/env python


import sys

import geometry_msgs.msg
import numpy as np

sys.path.append("/home/nooshin/projects/rbdl/build/python")
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import rbdl

model = rbdl.loadModel("/home/nooshin/Desktop/legRBDL.urdf")
g_joint_states = None
g_position = None
g_velocity = None
point_local = np.zeros(3)
point_local[0] = 0.0
point_local[1] = 0.0
point_local[2] = -0.2
pose = np.zeros(model.q_size)


def timer_callback(event):  # Type rospy.TimerEvent
    print('timer_cb (' + str(event.current_real) + '): g_positions is')
    print(str(None) if g_position is None else str(g_position))
    print()


def joint_callback(data):
    global g_joint_states, g_position
    rospy.loginfo(data.position)
    g_joint_states = data
    g_position = data.position
    g_velocity = data.velocity
    print(g_position)
    vel = rbdl.CalcPointVelocity(model, g_position, g_velocity, model.GetBodyId('calf'), point_local)
    print(str(vel))
    pose = rbdl.CalcBodyToBaseCoordinates(model, g_position, model.GetBodyId('calf'), point_local)
    print(pose)


def joint_logger_node():
    rospy.init_node('joint_logger_node', anonymous=True)
    rospy.Subscriber('joint_states', JointState, joint_callback)
    pub_pose = rospy.Publisher('rbdl_result', geometry_msgs.msg.Pose, queue_size=10)
    rospy.Timer(rospy.Duration(2), timer_callback)
    pub_pose.publish(pose)

    rospy.spin()


# sub = rospy.Subscriber('/joint_states',JointState,joint_callback)
# pub = rospy.Publisher('COM_pos',Float64,queue_size=10)

if __name__ == '__main__':
    joint_logger_node()
