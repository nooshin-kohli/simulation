'''
   Author : Nooshin Kohli
   Year : 2021-2022
'''
import numpy as np
import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/projects/rbdl/build/python'
sys.path.append(dir)
import rbdl


class ROBOT():
    def __init__(self, q, qdot, urdf_path):     # TODO: path to leg_RBDL.urdf for robot without slider and legRBDL.urdf for slider
        self.model = rbdl.loadModel(urdf_path)
        self.q = q
        self.qdot = qdot
        self.calf_length = -0.240
        self.end_point = np.asarray([0.0, 0.0, self.calf_length])
        self.fb_dim = 3                                                          # TODO: check this
        self.point_F_dim = 1                                                     # TODO: check this
        self.qdim = self.model.q_size
        self.S = np.hstack((np.zeros((self.qdim - self.fb_dim, self.fb_dim)), np.eye(self.qdim - self.fb_dim)))


    def calcJc(self, q):
        jc = np.zeros((3, self.qdim))
        rbdl.CalcPointJacobian(self.model, q, self.model.GetBodyId('calf'), self.end_point, jc)
        return jc

    def velocity_end(self, q, qdot):
        vel = rbdl.CalcPointVelocity(self.model, q, qdot, self.model.GetBodyId('calf'), self.end_point)
        return vel

    def pose_end(self, q):
        pose_base = rbdl.CalcBodyToBaseCoordinates(self.model, q, self.model.GetBodyId('calf'), self.end_point)
        return pose_base

    def pose_slider(self, q):
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, q, self.model.GetBodyId('jump'), np.array([0.0,0.0,0.00000]))
        return pose

    def CalcTau(self, q, qdot, qddot):
        Tau = np.zeros(self.model.q_size)
        rbdl.InverseDynamics(self.model, q, qdot, qddot, Tau)
        return Tau

    def endpose_BodyCoordinate(self, body_name, q):   # this function returns end point in any body coordinate you want
        pose_base = rbdl.CalcBodyToBaseCoordinates(self.model,q, self.model.GetBodyId('calf'), self.end_point)
        pose = rbdl.CalcBaseToBodyCoordinate(self.model, q, self.model.GetBodyId(body_name), pose_base)
        return pose

    def a_end(self,q, qdot, qddot):
        a_end = rbdl.CalcPointAcceleration(self.model, q, qdot, qddot, self.model.GetBodyId('calf'), self.end_point)
        a_end_world = rbdl.CalcBodyToBaseCoordinates(self.model, q, self.model.GetBodyId('calf'), a_end)
        return a_end_world

    def inv_kin(q,qdot,qddot,size,model):
        Tau = np.zeros(size)
        Tau = rbdl.InverseDynamics(model, q, qdot, qddot, Tau)
        return Tau

    def set_input(self, tau):
        if len(tau) == self.qdim: self.u0 = np.dot(self.S, tau)
        else: self.u0 = tau
        # print(self.u0)
#        print self.u0
        return None
    
    def CalcM(self,q):
        M = np.zeros((self.model.q_size,self.model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(self.model, q, M, True)
        # self.CalcM= M
        return M

    def Calch(self, q, qdot):
        h = np.zeros(self.model.q_size)
        rbdl.InverseDynamics(self.model, q, qdot, np.zeros(self.model.qdot_size), h)
        return h

    def ComputeContactForce(self, qqdot, p, u):
        q = qqdot[:self.qdim] 
        qdot = qqdot[self.qdim:]
        Jc = self.calcJc(q)
        # Jc = self.Jc_from_cpoints(self.model, q, self.body, p)                             # TODO: Nooshin get this function
        M = self.CalcM(q)   
        h = self.Calch(q, qdot)
        # print(M)
        self.ForwardDynamics(qqdot.flatten(), M, h, self.S, u, Jc, p)
        return None

    def SetGRF(self, p, values):                                            # TODO: what is this?
#        print 'yes', p
        last = 0
        if 1 in p:
            p_1 = last
            last += self.point_F_dim
        if 2 in p:
            p_2 = last
            last += self.point_F_dim
#        if 3 in p:
#            p_3 = last
#            last += 3
#        if 4 in p:
#            p_4 = last
#            last += 3
        self.Lambda = np.zeros(2*self.point_F_dim)*np.nan
        if 1 in p:
            self.Lambda[:self.point_F_dim] = values[p_1:p_1+self.point_F_dim]
        if 2 in p:
            self.Lambda[self.point_F_dim:2*self.point_F_dim] = \
            values[p_2:p_2+self.point_F_dim]
#        if 3 in p:
#            self.Lambda[6:9] = values[p_3:p_3+3]
#        if 4 in p:
#            self.Lambda[9:] = values[p_4:p_4+3]
        print("lambda is:")
        print(values.reshape(3,1))
        return None

    def CalcAcceleration(self, q, qdot, qddot, body_id, body_point):
        body_accel = rbdl.CalcPointAcceleration(self.model, q, qdot, qddot, body_id, body_point)
        return body_accel
    # cp = contact point
    def CalcGamma(self, cp, q, qdot):
        
        self.cbody_id = []
        
        if 1 in cp:
            for i in range(self.point_F_dim):self.cbody_id.append(\
            self.model.GetBodyId('calf'))
        
        
        Normal = []
        for i in range(len(cp)):
            Normal.append(np.array([1., 0.]))
            Normal.append(np.array([0., 1.]))
#            Normal.append(np.array([0., 0., 1.]))
        
        
        k = len(cp)*self.point_F_dim
        # Gamma = np.zeros(k)
        Gamma = np.zeros((3,1))
        prev_body_id = 0
        
        gamma_i = np.zeros(self.point_F_dim)
        
        for i in range(k):
            
            if prev_body_id != self.cbody_id[i]:
                gamma_i = rbdl.CalcPointAcceleration(self.model, q, qdot, np.zeros(self.qdim), self.cbody_id[i], self.end_point)                                         #TODO: [:2]?               
                prev_body_id = self.cbody_id[i]  
            Gamma = gamma_i
        return Gamma

    def test(self,q):
        print(self.CalcM(q))
        return None

    
    
    def ForwardDynamics(self, x, M, h, S, tau, Jc, cpoints):
        fdim = np.shape(Jc)[0]
        qdim = self.qdim
        q = x[:qdim]
        qdot = x[qdim:]

        
        if fdim == 0:
            self.qddot = np.dot(np.linalg.inv(M), np.dot(S.T, self.u0) - h).flatten()         
            self.Lambda = np.zeros(fdim)*np.nan
        else:
            
            
            if np.nonzero(qdot)[0].any() and Jc.any():
#                tic = time.time()
#                gamma = self.CalcGamma(cpoints, q, qdot)
                gamma = self.CalcGamma(cpoints, q, qdot)
#                print gamma - mygamma
#                toc = time.time() - tic
#                print toc
            else:
                gamma = - np.dot(np.zeros_like(Jc), qdot)
                    
            # print(gamma)
            aux1 = np.hstack((M, -Jc.T))
            aux2 = np.hstack((Jc, np.zeros((fdim, fdim))))
            A = np.vstack((aux1, aux2))
            B = np.vstack(((tau - h).reshape(qdim, 1), gamma.reshape(fdim, 1)))
            res = np.dot(np.linalg.inv(A), B).flatten()
            self.qddot = res[:-fdim]
            self.SetGRF(cpoints,  res[-fdim:])                              
            
            
#            print "======================================="            
#            print 'p, lambda:', self.__p[-1], self.Lambda
#            print "======================================="
        
        return None
        

# TODO: you can put print in every function you want to use:
# TODO: I put print in setGRF function for calculating lambda
q = np.zeros(4)
q[1] = np.pi/2
qdot = np.zeros(4)
qdot[2]= 0.3
robot = ROBOT(q, qdot, "/home/kamiab/catkin_ws/src/simulation/first_leg/scripts/legRBDL.urdf")
tau = np.zeros(4)
robot.set_input(tau)
# print(robot.CalcM(q))
# print(robot.Calch(q,qdot))
p = [1]
qqdot = np.zeros(8)
# print(robot.CalcGamma([1],q,qdot))
print(robot.ComputeContactForce(qqdot,p,tau))
# print(robot.calcJc(q))
# x_dot = np.array([0.0,0.0,0.0])
# j = robot.calcJc(q)
# j_sudo_inv = np.dot(np.transpose(j),np.linalg.inv(np.dot(j,np.transpose(j))))
# qdot_d = np.dot(j_sudo_inv,x_dot)





