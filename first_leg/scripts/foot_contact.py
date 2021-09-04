#! /usr/bin/env python3.6
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from VPend import VP


def callback(data):
    q = data.position
    q = np.asarray(q)
    qdot = data.velocity
    qdot = np.asarray(qdot)
    global pose
    r = VP(q, qdot)
    pose = r.pose
    if pose[2] == 0:
        print("robot touched ground!!")
    else:
        print(str(pose))



def main():
    rospy.init_node('publish_distance')
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()
    pub = rospy.Publisher('/distance', Int32MultiArray, queue_size=10)
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        pub.publish(pose)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass




