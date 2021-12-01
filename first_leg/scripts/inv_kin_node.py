#! /usr/bin/env python3.6 
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
Qi = np.array([0., 0., -pi/6])

def modelKin(q):
    J = robot.calcJc(q)
    p = robot.pose_end(q)
    return [p,J]


n = 3 
Posi,Ji = modelKin(Qi)
    # time step
dt  = 1e-3
tf  = 10

    # Trajectory Amplitude and frequency
A   = array([0.1, 0., 0.])
P0 = array([.21, .12, .4790539])
# w   = pi/4;          # the period is therefore 2pi/w
w = 0.5
k   = len(A);        # task space dimension

# ref Jacobian
#TODO
Qr  = np.array([0., 0., -pi/6])
# Qr = q_rbdl
Pos, J = modelKin(Qr)
J        = J[0:k,:] 

Jr = np.zeros(k)

for i in range(k):
    Jr[i]=norm(J[i,:])**2

t = 0
tpre = time.time()

# IK Gain
K_IK= 50          # IK close loop gain
D_IK= 0.001      # IK damping gain
MaxI= 10         # Maximum number of iteration

# Position limits
Qmax = 3*np.array([1., 1., 1.2])
Qmin = -Qmax

# Velocity limits
Qdmax = 5*np.array([1., 1., 1.])
v_eps = 1e-3
Qdmin = -Qdmax

# Acceleration limits
Qddmax = 10 * np.array([1., 1., 1.])
Qddmin = -Qddmax

# initializations
# t = 0                           # time
Q   = Qi                 # joint positions
Pos = Posi                      # End-effctor position

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

def get_state(data):
    global t,tpre
    global tV,TrajV,PosV,QdV,QV,CJNV,sV,jV,CV,Q    
    #for i in range(dim):
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
    # print(q_rbdl)
        # Trjajectory definition
    #    tra_per = 2*pi/w
    DP      = A * sin(w*t)
    # print(DP)
    Dpd     = A * w * cos(w*t)  
        
    Traj   = DP + P0
    # print(Traj)
    Q = q_rbdl
    # print(Q)
    
        # Jacobian/Position extraction
    Pos,J = modelKin(Q);
    # print(Pos)
    J      = J[:k,:]        # only position related block
       
        # initial wight 
    #   WM     = diag(MDi);%ones(1,n);
    WM = np.diag(np.ones(n))
    
    JJ      = J
    
    Pos	= Pos[:k]
    
    #     velocity vector
    Pos_error = Traj - Pos
    Xd      = Dpd + K_IK * Pos_error
    print(Xd)
    
        # update the joint limits
    Vmin   = np.max(np.vstack(((Qmin-Q)/dt,Qdmin,-sqrt(Qddmin * (Qmin-Q)))), axis=0)    
    Vmax   = np.min(np.vstack(((Qmax-Q)/dt,Qdmax, sqrt(Qddmax * (Qmax-Q)))), axis=0)    
    
    #     IK
    #     damping
    ED = 1 / 2 * (Pos_error.dot(Pos_error))
    # initialisation
    SMv = np.ones((1,n))
    QdN = np.zeros((1,n))
    s = 1
    RankJ = 0
    # first solution
    Jtw = np.dot(JJ , np.diag(SMv[0]))
    #%     Ji  = Jtw'/(ED * eye(2*k) + D_IK*diag([J1r,J2r]) + Jtw * Jtw'); 
    aa = np.dot(WM, np.transpose(Jtw)) 
    bb = ED * np.eye(k) + D_IK * np.diag(Jr)\
    + np.dot(Jtw, np.dot(WM, np.transpose(Jtw)))  

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
            
            aa = np.dot(WM, np.transpose(Jtw)) 
            bb = ED * np.eye(k) + D_IK * np.diag(Jr)\
                + np.dot(Jtw, np.dot(WM, np.transpose(Jtw)))  

            Ji = np.dot(aa, inv(bb))                  
            
        jV[i,0] = j + 1
        CJNV[i,j] = CJN;
        # dt = time.time() - tpre               #TODO check this
    # print(Q)
    print(Qd)
    # print(dt)    
    Q      = Q + Qd * dt
    # print(Q)
    pub_hip.publish(Q[0])
    pub_thigh.publish(Q[1])
    pub_calf.publish(Q[2])
    CV[i,:] = cond(Ji)
    TrajV[i,:] = Traj
    PosV[i,:] = Pos
    QV[i,:] = Q
    QdV[i,:] = Qd
    tV[i,:] = t
    sV[i,:] = s
    #    % time update
    t = t + dt;
    # tpre += dt
        

        




def main():
    rospy.Subscriber("/leg/joint_states", JointState, get_state)
    rospy.spin()
    if rospy.is_shutdown():
        plt.figure();
        plt.subplot(211)
        plt.plot(tV,TrajV,'-',tV,PosV,'--')
        plt.title('Desired and actual trajectories')
        plt.subplot(212)
        plt.plot(tV,np.abs(TrajV - PosV))
        plt.ylabel('error')
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

        # plt.show()




if __name__=="__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass