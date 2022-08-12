#! /usr/bin/env python3.6
from pickle import NONE
import numpy as np
from numpy.core.fromnumeric import transpose
import rospy
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
first_jump=1

torque_hip = []
torque_thigh = []
torque_calf = []

##################################importing the contact forces from slip model
# import test
from object import data_input
#####################  m=2.643
h = data_input(dt=.01, m=4, L0=0.398, k0=1500)
# print("ground Force_test:")
# print(h.function(3)[0][0]) 

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


GF_contact, y_des_contact = extract_data(h.function(3)[0], h.function(3)[1])
print(type(h.function(3)[1]))

##### interpolate y_des & GF_contact
tau_s = np.linspace(0, 1, len(GF_contact))
time_test= np.linspace (0, 3, len(h.function(3)[1]))
intp_gf = intp(tau_s, GF_contact, k=1)
intp_y = intp(tau_s, y_des_contact, k=1)
intp_y_comp =  intp(time_test, h.function(3)[1], k=1)
error_pre = [0,0,0,0]
def pose(q_rbdl,qdot_rbdl,delta_time,error_pre):
    qdot_d = np.zeros(4)
    Q_h = np.array([0, 0.0231, 0.6, -1.2]) # actual zero position of the robot
    Kp= [[0,0,0,0],
        [0,10,0,0],
        [0,0,20,0],
        [0,0,0, 1]]
        
    Kd= [[0, 0, 0, 0],
         [0, 0.1, 0, 0],
         [0, 0, 0.1, 0],
         [0, 0, 0, 0]]
    
    error_dot = qdot_d - qdot_rbdl
    
    error = Q_h - q_rbdl
    error_dot  = (error-error_pre)/delta_time
    error_pre = error
    
    #print(error)
    tau_in = np.dot(Kp, error) + np.dot(Kd, error_dot)

    #print(tau_in)
   # print(q_rbdl)
    pub_calf.publish(tau_in[3])
    pub_thigh.publish(tau_in[2])
    pub_hip.publish(tau_in[1])
    pub_slider.publish(0)

    torque_hip.append(0)
    torque_thigh.append(0)
    torque_calf.append(0)
    

    
########################################################  CONTACT MODE CONTROLLER  (COMPRESSION AND DECOMPRESSION)
e_pre =[0,0,0]
def contact(robot, delta_time, jc, GF, y_d, q_rbdl, qdot_rbdl,e_pre):

    p = 25
    K_p = [[p, 0, 0],
            [0, p, 0],
            [0, 0, p]]

    d= 2
    K_d = [[d, 0, 0],
            [0, d, 0],
            [0, 0, d]]
            
    k = 0       
    k_v = [[k, 0, 0],
            [0, k, 0],
            [0, 0, k]]


    j_COM = robot.computeJacobianCOM('slider',q_rbdl)
    COM = robot.get_com(calc_velocity=True,body_part='h',q=q_rbdl,qdot=qdot_rbdl)

    ################################# desired task space #################################
    desire_pos = np.array([COM[0][0], COM[0][1], y_d])
    # desire_vel = np.array([COM[1][0], COM[1][1], ydot_d])
    # desired_vel = desire_vel.reshape(3,1)

    ################################ calculating COM errors #################################
    e = desire_pos - COM[0]
    e_dot = (e - e_pre)/delta_time
    e_pre = e
    # e_vel = desire_vel - COM[1]

    ################################ task space to joint space ################################ 
    # tau_v = np.dot(j_COM.T,(np.dot(k_v,e_vel)))
    tau_pd = np.dot(j_COM.T,(np.dot(K_p,e)+np.dot(K_d, e_dot)))
    J_t = jc.T
    G_F=[0,0,-GF]
    Tau_ff = np.dot(J_t, G_F)
    tau = (Tau_ff - tau_pd).flatten()
    p_gain = np.dot(K_p,e)
    d_gain = np.dot(K_d, e_dot)
    print("Tau_ff: ", Tau_ff)
    print("tau_pd: ",tau_pd)
    pub_calf.publish(tau[3])
    pub_thigh.publish(tau[2])
    pub_hip.publish(tau[1]) 
    pub_slider.publish(0)
    return tau



##########################################################################################################  HOMING THE ROBOT AT HOME POSITION AT FIRST
def homing(q_rbdl, slider_h):
    #Q_h = np.array([0, -0.0231, 0.05, -0.9948]) # actual zero position of the robot
    #Q_h = np.array([0, 0.0231, 0.05, -0.3948])
    Q_h = np.array([0, 0.0231, 0.6, -1.2])

    Kp= [[0,0,0,0],
        [0,10,0,0],
        [0,0,1,0],
        [0,0,0,1]]
    # Kd= [[0,0,0,0],
    #     [0,0.9,0,0],
    #     [0,0,0.1,0],
    #     [0,0,0,1.5]]

    
    error_h= 1.1   - slider_h
    error= Q_h - q_rbdl
    # error_dot = np.zeros(4)-qdot_rbdl
    #print(error)
    #tau_in=np.dot(error,Kp)
    tau_in=np.dot(Kp,error)
    #print(tau_in)
   # print(q_rbdl)
    pub_calf.publish(tau_in[3])
    pub_thigh.publish(tau_in[2])
    pub_hip.publish(tau_in[1])
    if error_h > 0.6 or error_h < -0.6:
        pub_slider.publish(error_h*250)

    elif (error_h < 0.6 and error_h > 0.3) or (error_h > -0.6 and error_h< -0.3):
        pub_slider.publish(error_h*230)
    
    else:
        pub_slider.publish(error_h*200)
    print(slider_h)

def compute_tau(t_now, t_td, t_lo):
    tau = (t_now - t_td)/(t_lo - t_td)
    return tau
        

def mode_detect(x):
    if x[2] < 0.03:
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


sample_num=0
sample_vec=[]
force_vec=[]
feedback_vec=[]
height_vec=[]
time_vec=[]
slip_height_error=[]
real_time_vec=[]
t_real_first= 0
t_ros_first_fall= 0 
td_counter=0
cond = []
y_des_check =[]
time_check=[]
tpre = rospy.get_time()

def callback(data):
    global error_pre,e_pre,tpre, y_des, modeVec, GF_contact, y_des_contact ,groundForce, t_ros_pre, count ,dt, t_ros_first,i, switch_mode
    global td_counter, sample_vec, force_vec,t_real_first, feedback_vec, sample_num, height_vec, time_vec, t_td, t_lo, real_time_vec, t_ros_first_fall
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
    cond.append(np.linalg.cond(jc))
    
    x = robot.pose_end(q_rbdl)
    
    slider_h = robot.pose_slider(q_rbdl)
    slider_h = slider_h[2]
    
    
    t_now = time.time()
    delta_time = t_now - tpre
    tpre = t_now
    
    t_ros_now = rospy.get_time() - t_ros_first
    time_vec.append(t_ros_now)
    qqdot = np.concatenate((q_rbdl, qdot_rbdl), axis= 0)
    
    # if t_ros_now > 5:
    #      height_error = h.function(3)[1][int((time.time()-5-t_real_first)/0.01)]  - slider_h
    #      slip_height_error.append(height_error)
    #      time_vec.append( time.time()- t_real_first)
    # print(q_rbdl)
    # print (qdot_rbdl)
    # print("qqdot", qqdot)


    mode = mode_detect(x)
    #print(mode)
   # print(x)

    ###################################################
    if t_ros_now < 15 :
        homing(q_rbdl, slider_h)
        print("X:", x)
        print(slider_h)
        # print("+++++++++++++++++++++++++++++++++++++++++++++++++", delta_time)
       
    else:
        print("TD:",td_counter)
        pub_slider.publish(0)
        if count ==1 :
            t_real_first = time.time()
            t_ros_first_fall = rospy.get_time()
            count = 2

        # slip_height_error.append( slider_h - intp_y_comp(time.time()-t_real_first) )
        # real_time_vec.append(time.time() - t_real_first)
        slip_height_error.append( slider_h - intp_y_comp(rospy.get_time() - t_ros_first_fall) )
        real_time_vec.append(rospy.get_time() - t_ros_first_fall)

        if mode == 'contact': #or switch_mode == 0:
            if i==0 :
                t_td = rospy.get_time()
                t_lo = t_td + len(GF_contact) * 0.01
                td_counter+=1
                print("TOUCH DOWN")
            print("i:  ", i)
            print("t_td:", t_td)
            print("t_lo:", t_lo)
            print("time-now:", rospy.get_time())
            tau = compute_tau(rospy.get_time(), t_td, t_lo)
            print("tau:", tau)
            torque = contact(robot, delta_time, jc, intp_gf(tau), intp_y(tau),q_rbdl,qdot_rbdl,e_pre)
            y_des_check.append(intp_y(tau))
            time_check.append(rospy.get_time() - t_ros_first_fall)
            ################################ compute the feedback force 
            robot.set_input(torque)
            p = [1]
            feed_back_force = robot.ComputeContactForce(qqdot, p, torque)
            feedback_vec.append(feed_back_force[-1])
            force_vec.append(intp_gf(tau))
            sample_vec.append(rospy.get_time() - t_ros_first_fall)
            sample_num+=1
            # print("GF contact:",GF_contact[i])
            # print("feed_back:", feed_back_force)
            # print("torque:", torque)
            i+=1
            # print(i)
            # switch_mode=0
            # if (i>=len(GF_contact)):
            #     i=0
            #     switch_mode=1
            #     print("cycle change")
        else:
            pose(q_rbdl, qdot_rbdl,delta_time,error_pre)
            force_vec.append(0)
            sample_vec.append(rospy.get_time() - t_ros_first_fall)
            feedback_vec.append(0)
            i=0
            # if td_counter > 5:
            #     rospy.signal_shutdown("td_count")
        height_vec.append(slider_h)
    ############################################################ To find L0 uncomment below 
    # print("L0: ", slider_h - robot.pose_end(q_rbdl)[2])  
        # print("real time:", time.time() - t_real_first)
    
    


def main():
    # rospy.init_node('command', anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, callback)
    rospy.spin()
    #rospy.signal_shutdown(reason)
    if rospy.is_shutdown():
        plt.figure()
        plt.plot(sample_vec, feedback_vec)
        plt.plot(sample_vec, force_vec)
        plt.plot(np.linspace(0,3, num=len(h.function(3)[0])),h.function(3)[0])
        plt.legend(["feedback_force", "Force_from_model"], loc ="upper right")
        plt.title('feedback vec')
       

        plt.figure()
        plt.plot(sample_vec, height_vec)  
        plt.scatter(time_check, y_des_check)
        plt.plot(np.linspace(0,3, num=len(h.function(3)[1])),h.function(3)[1])     
        plt.legend(["slider-height", "y_input_desire","y_des_from_model"], loc ="upper right")
        # plt.figure()
        # plt.plot(time_vec, height_vec)
       
       
        # plt.figure()
        # plt.plot(sample_vec, torque_hip, 'r')
        # plt.plot(sample_vec, torque_thigh, 'g')
        # plt.plot(sample_vec, torque_calf, 'b')
        # plt.legend(["hip torque","thigh torque","calf torque"], loc ="upper right")

        # plt.figure()
        # plt.plot(time_vec,cond)

        ######################test
        # print(len(GF_contact))
        # plt.figure()
        # x = np.linspace(0, len(GF_contact))
    
        # plt.plot(x, GF_contact, 'r', lw=3, alpha=0.7)
        # plt.title('GF_contact')
        

        # plt.figure()
        # xs = np.linspace(0, 1, 1000)
        # plt.plot(xs, intp_gf(xs), 'r', lw=3, alpha=0.7)
        # plt.title('interpolated height')

        plt.figure()
        plt.plot(real_time_vec, slip_height_error)
        plt.show()
        #  pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass

####[0.17902869 0.09235515 0.2273961 ] home-position in x y z