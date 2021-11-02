#! /usr/bin/env python3.6
'''
Author: Nooshin Kohli

'''


import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from VPend import VP

pub = rospy.Publisher('/end_pose', Int32MultiArray, queue_size=10)
def callback(data):
    q = data.position
    q = np.asarray(q)
    qdot = data.velocity
    qdot = np.asarray(qdot)
    global pose
    r = VP(q, qdot)
    pose = r.pose
    for_pub = Int32MultiArray(data=pose)
    pub.publish(for_pub)
    if pose[2] == 0:
        print("robot touched ground!!")
    else:
        print(str(pose))



def main():
    rospy.init_node('publish_distance')
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass




