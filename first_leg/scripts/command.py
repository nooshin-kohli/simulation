#! /usr/bin/env python3.6
# author: kamiab yazdi  
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
import numpy as np

pub_1 = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)
pub_2 = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
pub_3 = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)

rospy.init_node('command', anonymous=True)
#tpre = rospy.get_time()
#suggest : rospy.Time.now().to_sec()
run_once=0
def home_position():
    pub_1.publish(-1.630959899263968)
    pub_2.publish(1.0126971578323032)
    pub_3.publish(0.05022888169204609)
    

tpast=time.time()
tpre=time.time()
first_check=0
i=0
def callback(data):
    global tpast
    global tpre
    global first_check
    global q_pre
    global P_d
   
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
    
    kp=[[50,0,0],
        [0,50,0],
        [0,0,50]]
    
    
    #-------------------------------------------------------------------JACOBIAN
    
    robot = ROBOT(q_rbdl, qdot_rbdl, "/home/nooshin/catkin_learn/src/first_leg/scripts/leg_RBDL.urdf")  # TODO: give your own urdf_path
    jc = robot.calcJc(q_rbdl)
    jc_inverse=np.linalg.inv(jc)
    #print(np.linalg.cond(jc))
    #print("Jacobian inverse:\n")
    #print(jc_inverse)
    #------------------------------------------------------------------END POSITION
    position=robot.pose_end(q_rbdl)
    position[0]-=0.03
    position[2]-=0.899
    # print('End position:\n')
    # print(position)
    
    if first_check==0:
        q_pre=q_rbdl
        first_check=1
        #home position values:
        # pub_1.publish(-1.630959899263968)
        # pub_2.publish(1.0126971578323032)
        # pub_3.publish(0.05022888169204609)
        # t_pass=time.time()
        # while ((time.time() - t_pass)< 5):pass
    q_pre=q_rbdl
           
    #----------------------------------------------------------------DESIRE POSITION APPROACH
    inst=time.time()-tpast
    P_d=[0.2*np.sin(np.pi*inst),0.1,-0.275]
    dp_d=[0.2*np.cos(np.pi*inst),0,0]
    #dp_d=[0,0,0]
    #P_d=[-0.26,0.1,-0.28]
    #P_d=[0.25,0.1*np.sin(np.pi*time.time()),-0.28]
    #P_d=[0.25,0.2*np.sin(np.pi*time.time()-tpast),-0.28]
    #P_d=[1,1,1]
    #P_d=[0.5*np.sin(np.pi*time.time()),0.1,-0.28]
    error=P_d-position
    # print("error:\n")
    print(error)
#-------------------------------------------------------------------CALCULATE Q_d
    
   
    Pdot_d=dp_d+np.dot(error,kp)
    qdot_d=np.dot(jc_inverse,Pdot_d)
    dt = time.time() - tpre
    tpre += dt
    q_d=q_pre+qdot_d*dt
    #q_pre=q_d
   # print(q_d)
    diff= q_d-q_pre
   # print(np.linalg.cond(jc))
    pub_1.publish(q_d[2])
    pub_2.publish(q_d[1])
    pub_3.publish(q_d[0])
    
    # pub_1.publish(-1.630959899263968)
    # pub_2.publish(1.0126971578323032)
    # pub_3.publish(0.05022888169204609)
    #print(diff)
    # print("-----------------------------------------------------------------------------------") 
    # if np.linalg.cond(jc) < 12:
    #     pub_1.publish(q_d[2])
    #     pub_2.publish(q_d[1])
    #     pub_3.publish(q_d[0])
    # else:
    #     pub_1.publish(0.1)
    #     pub_2.publish(0.1)
    #     pub_3.publish(0.1)
    #     sys.exit["unsafe command!!"]
    
  
       
    
   #--------------------------------------------------------for plot
    val_x_list=[P_d[0],P_d[1],P_d[2],position[0],position[1],position[2],time.time()-tpast]
    # val_x_list.append(P_d[0])
    # val_x_list.append(P_d[1])
    # val_x_list.append(P_d[2])
    # val_x_list.append(position[0])
    # val_x_list.append(position[1])
    # val_x_list.append(position[2])
    # val_x_list.append(time.time()-tpast)
    # val_x_list = np.asarray(val_x_list)
    #print(val_x_list)


#    while ((rospy.get_time() - tpre)< 0.001):pass
#    tpre = rospy.get_time()
#    while ((time.time() - tpast)< 0.005):pass
#    tpast=time.time()


def main():   
    # rospy.init_node('command', anonymous=True)
        rospy.Subscriber("/leg/joint_states", JointState, callback)
        rospy.spin()
      


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
