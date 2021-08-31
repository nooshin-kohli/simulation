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
    #print (q)
    #print(qdot)
    global vel
    r = VP(q, qdot)
    vel = r.vel
    print(str(r.vel))



def main():
    rospy.init_node('publish_vel')
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()
    pub = rospy.Publisher('/velocity', Int32MultiArray, queue_size=10)
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        pub.publish(vel)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass




