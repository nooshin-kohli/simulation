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
data_list=[]
x_list=[]
xd_list=[]
y_list=[]
yd_list=[]
zd_list=[]
z_list=[]
t_list=[]
error_x_list=[]
error_y_list=[]
error_z_list=[]
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
    
    robot = ROBOT(q_rbdl, qdot_rbdl, "/home/kamiab/catkin_ws/src/simulation/first_leg/scripts/leg_RBDL.urdf")  # TODO: give your own urdf_path
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
        



    q_pre=q_rbdl
           
    #----------------------------------------------------------------DESIRE POSITION APPROACH
    inst=time.time()-tpast
    P_d=[0.2*np.sin(0.5*np.pi*inst),0.1,-0.275]
    dp_d=[0.2*0.5*np.pi*np.cos(0.5*np.pi*inst),0,0]
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
    #print(q_d)
    diff= q_d-q_pre
   # print(np.linalg.cond(jc))
    pub_1.publish(q_d[2])
    pub_2.publish(q_d[1])
    pub_3.publish(q_d[0])
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
    x_list.append(position[0])
    xd_list.append(P_d[0])
    y_list.append(position[1])
    yd_list.append(P_d[1])
    z_list.append(position[2])
    zd_list.append(P_d[2])
    t_list.append(inst)
    error_x_list.append(error[0])
    error_y_list.append(error[1])
    error_z_list.append(error[2])

#    while ((rospy.get_time() - tpre)< 0.001):pass
#    tpre = rospy.get_time()
#    while ((time.time() - tpast)< 0.005):pass
#    tpast=time.time()


def main():   
    # rospy.init_node('command', anonymous=True)
        rospy.Subscriber("/leg/joint_states", JointState, callback)
        rospy.spin()
        if rospy.is_shutdown():
            print("ROS is SHUTDOWN!!")
            #############################################################################POSITION PLOT
            plt.plot(t_list,x_list,linestyle='-',color='red',label='x_position')
            plt.plot(t_list,xd_list,linestyle='--',color='b',label='x_d')
            plt.plot(t_list,y_list,linestyle='-',color='blue',label='y_position')
            plt.plot(t_list,yd_list,linestyle='--',color='b',label='y_d')
            plt.plot(t_list,z_list,linestyle='-',color='y',label='z_position')
            plt.plot(t_list,yd_list,linestyle='--',color='b',label='z_d')
            
            plt.plot()
            plt.legend()
            plt.title('K_p=50, x=(0.2)sin(0.5*pi.t),y=0.1, z=-0.275')
            plt.xlabel('time')
            plt.ylabel('position')
            #plt.xlim([0,3.5])
            plt.show()

            ###############################################################################ERROR PLOT
            # plt.plot(t_list,error_x_list,linestyle='-',color='red',label='error_x')
            # plt.plot(t_list,error_y_list,linestyle='-',color='yellow',label='error_y')
            # plt.plot(t_list,error_z_list,linestyle='-',color='blue',label='error_z')
            
            
            # plt.plot()
            # plt.legend()
            # plt.title('K_p=50, x=(0.2)sin(0.5*pi.t),y=0.1, z=-0.275')
            # plt.xlabel('TIME(s)')
            # plt.ylabel('ERROR(m)')
            # plt.xlim([1,7])
            # plt.show()
      


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
