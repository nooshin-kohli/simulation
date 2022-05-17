#! /usr/bin/env python3.6
from pickle import NONE
import numpy as np
from numpy.core.fromnumeric import transpose
import rospy
from scipy.signal import find_peaks
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT
from scipy.interpolate import InterpolatedUnivariateSpline as intp
import math
import time
import matplotlib.pyplot as plt
import sys
pub_hip = rospy.Publisher('/leg/hip_joint_effort_controller/command', Float64, queue_size=1000)
pub_thigh = rospy.Publisher('/leg/thigh_joint_effort_controller/command', Float64, queue_size=1000)
pub_calf = rospy.Publisher('/leg/calf_joint_effort_controller/command', Float64, queue_size=1000)
pub_slider = rospy.Publisher('/leg/jumper_effort_controller/command', Float64, queue_size=1000)
rospy.init_node('command', anonymous=True)
t_ros_pre = 0
t_ros_first=0
t_end=3
tVec=np.arange(0, 3, 0.01)
count=0
dt=0.01
i=0
switch_mode=1
t_td=0
t_lo=0.37
x_c = []
y_c = []
num_c = 0


##################################importing the contact forces from slip model
# import test
from object import data_input
h = data_input(dt=.01,m=2.643,L0=0.445,k0=300)
print("ground Force_test:")
print(h.function(3)[0][0]) 


##################################this function extract first step of contact forces
def extract_data(input_f, input_h):
    GF_contact=[]
    y_contact=[]
    for i in range(len(input_f)):
        if input_f[i]<0:
            pass
        else:
            while(input_f[i]>0):
                GF_contact.append(input_f[i])
                y_contact.append(input_h[i])
                i+=1
            break
    return GF_contact, y_contact


GF_contact, y_des_contact = extract_data(h.function(3)[0],h.function(3)[1])

##### interpolate y_des & GF_contact
tau_s = np.linspace(0, 1, len(GF_contact))
intp_gf = intp(tau_s, GF_contact, k=1)
intp_y = intp(tau_s, y_des_contact, k=1)


def pose(q_rbdl,qdot_rbdl):
    qdot_d = np.zeros(4)
    Q_h = np.array([0, 0.0231, 0.1, -0.3]) # actual zero position of the robot
    Kp= [[0,0,0,0],
        [0,15,0,0],
        [0,0,5,0],
        [0,0,0,2]]
    Kd= [[0,0,0,0 ],
        [0,0.5,0,0],
        [0,0.0,1,0],
        [0,0,0,0]]
    
    error_dot = qdot_d - qdot_rbdl
    
    error= Q_h - q_rbdl
    #print(error)
    tau_in=np.dot(error,Kp) + np.dot(error_dot,Kd)

    #print(tau_in)
   # print(q_rbdl)
    pub_calf.publish(tau_in[3])
    pub_thigh.publish(tau_in[2])
    pub_hip.publish(tau_in[1])
    pub_slider.publish(0)

    
#########################################################################################################  CONTACT MODE CONTROLLER  (COMPRESSION AND DECOMPRESSION)

def contact(slider_h, jc,GF,y_d,q_rbdl):

    Kp= [[0, 0, 0, 0],
         [0, 1,0,0],
         [0, 0,1,0],
         [0,0,0, 1]]
    e = y_d - slider_h

    gain = [0, 0, 0, e]
    J_t = jc.T
    
    G_F=[0,0,-GF]
    #print(GF)
    Tau_ff = np.dot(J_t , G_F)
    # print("tauff",Tau_ff)
    K_p = [[0, 0, 0, 0],
           [0, 5, 0, 0],
           [0, 0, 5, 0],
           [0, 0, 0, 5]]
    print(np.dot(gain, K_p))
    Tau_c = np.dot(Kp,Tau_ff) + np.dot(gain, K_p)  #k_d * (e_dot) 
    print('tau_c=', Tau_c)
    pub_calf.publish(Tau_c[3])
    pub_thigh.publish(Tau_c[2])
    pub_hip.publish(Tau_c[1]) 
    pub_slider.publish(0)

    return Tau_ff

##########################################################################################################  HOMING THE ROBOT AT HOME POSITION AT FIRST
def homing(q_rbdl, slider_h):
    #Q_h = np.array([0, -0.0231, 0.05, -0.9948]) # actual zero position of the robot
    #Q_h = np.array([0, 0.0231, 0.05, -0.3948])
    Q_h = np.array([0, 0.0231, 0.1, -0.3])
    Kp= [[0,0,0,0],
        [0,15,0,0],
        [0,0,1,0],
        [0,0,0,1.5]]
    
    error_h= 1   - slider_h
    error= Q_h - q_rbdl
    #print(error)
    #tau_in=np.dot(error,Kp)
    tau_in=np.dot(Kp,error)
    #print(tau_in)
   # print(q_rbdl)
    pub_calf.publish(tau_in[3])
    pub_thigh.publish(tau_in[2])
    pub_hip.publish(tau_in[1])
    if error_h > 0.8 or error_h < -0.8:
        pub_slider.publish(error_h*250)

    elif (error_h < 0.8 and error_h > 0.3) or (error_h > -0.8 and error_h< -0.3):
        pub_slider.publish(error_h*230)
    
    else:
        pub_slider.publish(error_h*200)
    print(slider_h)

def compute_tau(t_now, t_td, t_lo):
    tau = (t_now - t_td)/(t_lo - t_td)
    return tau
        

def mode_detect(x):
    if x[2]<0.025:
        mode = 'contact'
        print("contact")
    else :
        mode = 'fly'
        print("flight")
    return mode



def sub_f_t(data):
    global force,torque
    force = data.wrench.force
    # print(type(force_left))
    force[0] = force.x
    force[1] = force.y
    force[2] = force.z
    # print(force)
    torque = data.wrench.torque
    torque[0] = torque.x
    torque[1] = torque.y
    torque[2] = torque.z

ax = plt.axes(projection='3d')
sample_num=0
sample_vec=[]
force_vec=[]
feedback_vec=[]
height_vec=[]
time_vec=[]

def callback(data):
    global tpre, y_des, modeVec, GF_contact, y_des_contact ,groundForce, t_ros_pre, count ,dt, t_ros_first,i, switch_mode
    global sample_vec, force_vec, feedback_vec, sample_num, height_vec, time_vec, t_td, t_lo, x_c, y_c, num_c
    if count==0:
        t_ros_first=rospy.get_time()
        switch_mode=1
        count=1
    q = data.position
    q = np.asarray(q)
    q_rbdl = np.zeros(4)
    q_rbdl[0] = q[2]    #slider height (origin 0.9)
    q_rbdl[1] = q[1]    #hip joint
    q_rbdl[2] = q[3]    #thigh joint
    q_rbdl[3] = q[0]    # calf joint

    #q_rbdl=[slider, hip, thigh, calf]


    qdot = data.velocity
    qdot = np.asarray(qdot)
    qdot_rbdl = np.zeros(4)
    qdot_rbdl[0] = qdot[2]
    qdot_rbdl[1] = qdot[1]
    qdot_rbdl[2] = qdot[3]
    qdot_rbdl[3] = qdot[0]
    
    robot = ROBOT(q_rbdl, qdot_rbdl,'slider')  # TODO: give your own urdf_path
    
    jc = robot.calcJc(q_rbdl)
    
    x = robot.pose_end(q_rbdl)
   
    
    slider_h = robot.pose_slider(q_rbdl)
    slider_h = slider_h[2]
    
    height_vec.append(slider_h)
    

    #t_now = time.time()-tpre
    t_ros_now = rospy.get_time() - t_ros_first
    time_vec.append(t_ros_now)
    qqdot = np.concatenate((q_rbdl, qdot_rbdl), axis= 0)
    
    # print(q_rbdl)
    # print (qdot_rbdl)
    # print("qqdot", qqdot)


    mode = mode_detect(x)
    #print(mode)
   # print(x)
    ###################################################
    if t_ros_now < 5 :
        homing(q_rbdl,slider_h)
       
    else:
        pub_slider.publish(0)
        if mode == 'contact': #or switch_mode == 0:
            num_c = num_c+1
            if i==0 :
                t_td = rospy.get_time()
                t_lo = t_td + 0.37
            end_pos = robot.pose_end(q_rbdl)
            x_c.append(end_pos[0])
            y_c.append(end_pos[1])

            print("t_td:", t_td)
            print("t_lo:", t_lo)
            tau = compute_tau(rospy.get_time(), t_td, t_lo)
            print("tau:", tau)
            torque = contact(slider_h, jc, intp_gf(tau), intp_y(tau),q_rbdl)

            
            ################################ compute the feedback force 
            robot.set_input(torque)
            p = [1]
            feed_back_force = robot.ComputeContactForce(qqdot, p, torque)
            feedback_vec.append(feed_back_force[-1])
            force_vec.append(GF_contact[i])
            sample_vec.append(sample_num)
            sample_num+=1
            # print("GF contact:",GF_contact[i])
            # print("feed_back:", feed_back_force)
            # print("torque:", torque)
            i+=1
            # print(i)
            switch_mode=0
            if (i>=len(GF_contact)):
                i=0
                switch_mode=1
                print("cycle change")
        else:
            pose(q_rbdl,qdot_rbdl)
            i=0
           
    

    


def main():
    # rospy.init_node('command', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()
    if rospy.is_shutdown():
        plt.figure()
        plt.plot(sample_vec, feedback_vec)
        plt.plot(sample_vec, force_vec)
        plt.legend(["feedback_force", "Force_from_model"], loc ="upper right")
        plt.title('feedback vec')
        plt.figure()
        # data = np.array([time_vec, height_vec])
        # print(type(data))
        # plt.plot(time_vec, height_vec)
        peaks, _ = find_peaks(height_vec, distance=1)
        # difference between peaks is >= 150
        print(np.diff(peaks))
        # prints [186 180 177 171 177 169 167 164 158 162 172]

        plt.plot(time_vec, height_vec)
        # print(type(height_vec))
        # plt.plot(peaks, height_vec[peaks], "x")
        plt.show()


        ######################test
        plt.figure()
        xs = np.linspace(0, 1, 1000)
        plt.plot(xs, intp_gf(xs), 'g', lw=3, alpha=0.7)
        plt.title('interpolated force')
        

        plt.figure()
        xs = np.linspace(0, 1, 1000)
        plt.plot(xs, intp_y(xs), 'r', lw=3, alpha=0.7)
        plt.title('interpolated height')

        # print("height:")
        # print(len(height_vec))

        print("x:")
        print(len(x_c))
        print(num_c)

        plt.figure()
        plt.plot(x_c,y_c)
        # ax.scatter3D(x_c,y_c,height_vec)
        plt.title('contact pose')

        plt.show()
        #  pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass

####[0.17902869 0.09235515 0.2273961 ] home-position in x y z



