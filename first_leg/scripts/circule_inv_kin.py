#! /usr/bin/env python3.6 

# AUTHOR : kohli , yazdi

import numpy as np
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT
import time
from numpy import sin, cos, pi, zeros, array, sqrt
from numpy.linalg import norm, inv, cond
from numpy.linalg import matrix_rank as rank
import matplotlib.pyplot as plt


pub_calf = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)
pub_thigh = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
pub_hip = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)
robot = ROBOT(np.zeros(3), np.zeros(3), "/home/kamiab/catkin_ws/src/simulation/first_leg/scripts/leg_RBDL.urdf")  # TODO: give your own urdf_path
    
rospy.init_node('command', anonymous=True)
import sys
import os
userdir = os.path.expanduser('~')
sys.path.append(userdir+"/projects/actuator")
# from actuator import Actuator
# from homing import home

sys.path.append(userdir+"/projects/rbdl3/build/python")
import rbdl
from robot_class import ROBOT
import time
import numpy as np
from numpy import sin, cos, pi, zeros, array, sqrt
from numpy.linalg import norm, inv, cond
from numpy.linalg import matrix_rank as rank
import matplotlib.pyplot as plt
plt.close('all')


def modelKin(q):
    J = robot.calcJc(np.array(q))
    p = robot.pose_end(np.array(q))
    return [p, J]


n   = 3 # number of links
path =  "/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf"
robot = ROBOT(np.zeros((3, 1)), np.zeros((3, 1)), path) #model


# q = np.array([0., 0., -pi/6])
q = np.array([-0.0231,0,-0.9948])   #TODO new home position of the robot
qr_pre = q
q_home = q
x = robot.pose_end(np.array(q))
home_pos  = x
print ("Homing position", home_pos)

Posi,Ji = modelKin(q)

dt = 0.008
tf  = 30
k   = 3   # task space dimension

tvec  = np.arange(0, tf, dt)
# print(tvec)
# w = np.pi
w = 0.5*np.pi
# w = 0.25*np.pi
A = .1
# xxd = home_pos[0] + A*np.sin(w*tvec ) - .02
xxd_x = home_pos[0] + A*np.sin(w*tvec )-0.01
# xxd = home_pos[0] + A*np.sin(w*tvec ) - .1
xxd_y = home_pos[1] + A*np.cos(w*tvec)-0.03
# xxd = home_pos[0] + A*np.sin(w*tvec ) 
dxxd_x = A*w*np.cos(w*tvec)
dxxd_y = -A*w*np.sin(w*tvec)

r = 0.05

tpre = time.time()

# ref Jacobian
#TODO
Qr  = q_home
Pos, J = modelKin(Qr)
J      = J[0:k,:] 
Jr = np.zeros(k)

for i in range(k):
    Jr[i]=norm(J[i,:])**2

Jr_diag = np.diag(Jr)


# IK Gain
K_IK= 1.         # IK close loop gain
D_IK= .001     # IK damping gain
MaxI= 10         # Maximum number of iteration

# # Position limits
Qmax = np.array([q_home[0] + pi/2, q_home[1] + pi/4, q_home[2] + pi/4])
Qmin = np.array([q_home[0] - pi/2, q_home[1] - pi/4, q_home[2] - pi/4])
# Qmax = np.array([q_home[0] + pi, q_home[1] + pi, q_home[2] + pi])
# Qmin = np.array([q_home[0] - pi/2, q_home[1] - pi/4, q_home[2] - pi/4])

# Velocity limits
Qdmax = 1*np.array([1., 1., 1.])
# Qdmax = 0.2*np.array([1., 1., 1.])
v_eps = 1e-3
Qdmin = -Qdmax;

# Acceleration limits
Qddmax = 10 * np.array([1., 1., 1.])
Qddmin = -Qddmax


# IK variables definitions
Smin    = zeros((1,n))
Smax    = zeros((1,n))

# Saving Variables
dim = int(tf/dt)
TrajV  = zeros((dim,k))
PosV   = zeros((dim,k))
QV      = zeros((dim,n))
QdV     = zeros((dim,n))
CV      = zeros((dim,1))
sV      = zeros((dim,1))
tV      = zeros((dim,1))
jV      = zeros((dim,1))
CJNV    = zeros((dim,MaxI))   # critical joint number

WM = np.diag(np.ones(n))

def IK(Q, Traj, Dpd):
    
    # Jacobian/Position extraction
    Pos,J = modelKin(Q);
    J      = J[:k,:]        # only position related block    
    JJ      = J
    Pos	= Pos[:k]
        
#     velocity vector
    Pos_error = Traj - Pos
    Xd = Dpd + K_IK * Pos_error
    
    
    # update the joint limits
    Vmin   = np.max(np.vstack(((Qmin-Q)/dt,Qdmin,-sqrt(Qddmin * (Qmin-Q)))), axis=0)    
    Vmax   = np.min(np.vstack(((Qmax-Q)/dt,Qdmax, sqrt(Qddmax * (Qmax-Q)))), axis=0)
    # print(Vmax)    
    
#     IK
#     damping
    ED = 1 / 2 * (Pos_error.dot(Pos_error))
    # initialisation
    SMv = np.ones((1,n))
    QdN = np.zeros((1,n))
    s = 1
    # first solution
    Jtw = np.dot(JJ , np.diag(SMv[0]))
#%     Ji  = Jtw'/(ED * eye(2*k) + D_IK*diag([J1r,J2r]) + Jtw * Jtw'); 
    Jtw_T = np.transpose(Jtw)
    aa = np.dot(WM, Jtw_T) 
    bb = ED * np.eye(k) + D_IK * Jr_diag + np.dot(Jtw, aa)  

    Ji = np.dot(aa, inv(bb))    

    for j in range(MaxI):
        Qdg = s* np.dot(Ji, np.transpose(Xd))
        Qdn = (QdN.T + np.dot(Ji , np.dot( - JJ , QdN.T))).flatten()
        Qd  = Qdg + Qdn
        for ii in range(n):
            if Qd[ii] > Vmax[ii] + v_eps:
                Smax[0,ii] = (Vmax[ii] - Qdn[ii])/Qdg[ii]
            elif Qd[ii] < Vmin[ii] - v_eps:
                Smax[0,ii] = (Vmin[ii] - Qdn[ii])/Qdg[ii];
            else:
                Smax[0,ii] =  np.inf; # it can be any number bigger than 1, e.g. 2
             

        CJN = np.argmin(Smax[0])
        sout = Smax[0, CJN]
        
#%         when there should be a saturated joint
        if sout<1:
#%             Saturating the velocity of Critiical Joint
            SMv[0,CJN] = 0
            if Qd[CJN] > Vmax[CJN]:
                QdN[0, CJN] = Vmax[CJN]
            else:
                QdN[0, CJN] = Vmin[CJN]

       
        if sout>1:  #% there is no limit exceedance 
#%             print('redundancy successfully limited the velocity')
            break
#            pass
#            %%%%%%% to be updated based on: initial rank of J and W update !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        elif rank(np.dot(JJ,np.diag(SMv[0]))) < k:
            SMv[0,CJN] = 1
            QdN[0,CJN] = 0
            s = sout
            #%             print('redundancy cannot limit the velocity, so the velocity is scaled')
        else:
#            % Jacobian update

#            %% Jacobian update
            Jtw = np.dot(JJ , np.diag(SMv[0]))
#            %     Ji  = Jtw'/(ED * eye(2*k) + D_IK*diag([J1r,J2r]) + Jtw * Jtw');         
            Jtw_T = np.transpose(Jtw)
            aa = np.dot(WM, Jtw_T) 
            bb = ED * np.eye(k) + D_IK * Jr_diag + np.dot(Jtw, aa)  

            Ji = np.dot(aa, inv(bb))                  
            
        jV[i,0] = j + 1
        CJNV[i,j] = CJN;
    
    return [Qd, s, Ji, Pos]
i = 0
def get_state(data):
    global t,tpre,xxd_y,dxxd_y,qr_pre,i,xxd_x,dxxd_x
    global tV,TrajV,PosV,QdV,QV,CJNV,sV,jV,CV,Q    
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

    xd = np.array([xxd_x[i], xxd_y[i], home_pos[2]])
    # print(xd)
   # xd = np.array([xxd[i], .0862, -.32])
    dxd = np.array([dxxd_x[i], dxxd_y[i], 0])
    
    dqr, s, Ji, Pos = IK(q_rbdl, xd, dxd)  
    # print(dqr)
    qr = dqr*dt + qr_pre
    # qr = -qr
    # print(qr)
    
    diff = np.abs(qr - q_rbdl)
    # print(diff)

    qr_pre = qr
    # print(qr)

    pub_hip.publish(qr[0])
    pub_thigh.publish(qr[1])
    pub_calf.publish(qr[2])

    x = robot.pose_end(np.array(qr))
    print(sqrt(x[0]**2+x[1]**2))
    
    #    %% Data saving
    CV[i,:] = cond(Ji)
    TrajV[i,:] = xd
    PosV[i,:] = x
    QV[i,:] = q_rbdl
    QdV[i,:] = dqr
#     tV[i,:] = t
    tV = tvec
    sV[i,:] = s
    i+=1
    
    tnow = time.time()
    # if tnow - tpre > dt:
    #     print ("Warning! actual step time is bigger than dt!")
    #     print ("It is: ", tnow - tpre, " at time = ", tnow)
    # while(time.time() - tpre < dt): temp = 0
    # tpre = time.time()
 

def main():
    rospy.Subscriber("/leg/joint_states", JointState, get_state)
    rospy.spin()
    if rospy.is_shutdown():
        plt.figure();
        plt.subplot(211)
        print("Traj:",TrajV)
        print("pos:",PosV)
        plt.plot(tV,TrajV,'-',tV,PosV,'--')
        plt.title('Desired and actual trajectories')
        plt.subplot(212)
        plt.plot(tV,np.abs(TrajV - PosV))
        plt.ylabel('error')
        ##
        plt.figure()
        plt.plot(PosV[:,0],PosV[:,1])
        plt.title('joint Velocities')
        ##
        plt.figure()
        plt.plot(tV,QdV)
        plt.title('joint Velocities')
        ##
        plt.figure()
        plt.plot(tV,QV,'-')
        plt.title('joint position')
        #plt.legend('q1','q2','q3','q4','q5','q6','q7')
        ##
        plt.figure()
        plt.plot(tV,CJNV)
        plt.title('Critical Joint Number')
        ##
        plt.figure()
        plt.plot(tV,sV)
        plt.title('Scaling factor')
        ##
        plt.figure()
        plt.plot(tV,jV)
        plt.title('Number of iterations')
        ##
        plt.figure()
        plt.plot(tV,CV)
        plt.title('Jacobian Condition Number')
        
        plt.show()
        



if __name__=="__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
