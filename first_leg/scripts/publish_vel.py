#! /usr/bin/env python3.6
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from VPend import VP

pub = rospy.Publisher('/velocity', Int32MultiArray, queue_size=10)

def callback(data):
    q = data.position
    q = np.asarray(q)
    qdot = data.velocity
    qdot = np.asarray(qdot)
    r = VP(q, qdot)
    vel = r.vel
    vel_pub = Int32MultiArray(data=vel)
    pub.publish(vel_pub)
    print(str(r.vel))



def main():
    rospy.init_node('publish_vel')
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass


