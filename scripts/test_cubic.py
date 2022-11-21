import matplotlib.pyplot as plt
import numpy as np
from robot_class import ROBOT
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

def jump(velocity,robot,t_td,t_d,q_first,t):
    T = t_d - t_td
    Tau = (t-t_td)/T
    q_end = np.array([0.022,0.7321,-1.05])
    # q_end = np.array(rbdl2robot(0.022,0.7321,-1.05)) # this hard code is from liftoff in slip model
    # J = robot.calcJc(q_end)
    J = robot.calcJcom(q_end)
    # print(np.linalg.cond(J))
    qdot_end = np.dot(np.linalg.inv(J),velocity)
    if Tau>1:
        raise ValueError("tau > 1 in decomp")
    q_first = np.array(q_first)
    qdot_first = np.zeros(3)
    # qdot_end = np.zeros(3)
    delta_q = q_end - q_first
    rho_0 = 0
    rho_1 = (T*qdot_first)/delta_q
    rho_2 = (-2*qdot_first-qdot_end)*T/delta_q + 3
    rho_3 = (qdot_first+qdot_end)*T/delta_q -2
    q = q_first + delta_q*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    return q

time = []
path = '/home/lenovo/python_simulation/python_simulation/leg_RBDL.urdf'
robot = ROBOT(np.zeros(3),np.zeros(3),path)
vel = [0, 0, 0]
vel = np.array(vel)
time_vec = np.linspace(0,0.1,100)
t_td = 0
t_d = 0.1
q_first = np.array([0.032, 1.2014, -1.819])
# q_first = np.array(rbdl2robot(0.032, 1.2014, -1.819))
q_vec = []
for t in time_vec:
    q = jump(vel, robot, t_td, t_d, q_first, t)
    q_vec.append(q)
    # y = cubic_comp(t, 0.668, 0.763, -0.366)
    # height.append(y)
    time.append(t)
q_vec = np.array(q_vec)
# q_vec_2 = np.array(q_vec_2)
plt.figure()
plt.plot(time, q_vec[:, 0])
plt.plot(time, q_vec[:, 1])
plt.plot(time, q_vec[:, 2])
# plt.plot(time, q_vec_2[:, 0])
# plt.plot(time, q_vec_2[:, 1])
# plt.plot(time, q_vec_2[:, 2])
plt.title("q cubic")
plt.legend(["hip","thigh","calf"],loc = "upper left")
# plt.show()


pub_calf = rospy.Publisher('/leg/calf_joint_position_controller/command', Float64, queue_size=10)
pub_thigh = rospy.Publisher('/leg/thigh_joint_position_controller/command', Float64, queue_size=10)
pub_hip = rospy.Publisher('/leg/hip_joint_position_controller/command', Float64, queue_size=10)

rospy.init_node("cammand", anonymous = 'True')
i = 0

def get_state(data):
    global q_home,model,i,q_vec  
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

    pub_hip.publish(q_vec[:, 0][i])
    pub_thigh.publish(q_vec[:, 1][i])
    pub_calf.publish(q_vec[:, 2][i])
    J_com = robot.calcJcom(q_rbdl)
    velocity = np.dot(J_com,qdot_rbdl)
    print("COM velocity: ", velocity)
    # for i in range(len(q_vec[:, 0])):
    i = i+1    


def main():
    rospy.Subscriber("/leg/joint_states", JointState, get_state)
    rospy.spin()
    if rospy.is_shutdown():
        # plt.figure()
        # plt.plot(h)
        # plt.show()
        print("DOWN")




if __name__=="__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass


