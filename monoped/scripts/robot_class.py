'''
   Author : Nooshin Kohli
   Year : 2021-2022
'''
from doctest import Example
import numpy as np
import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/rbdl/build/python'
sys.path.append(dir)
import rbdl


class ROBOT():
    def __init__(self, q, qdot, mode):     # TODO: path to leg_RBDL.urdf for robot without slider and legRBDL.urdf for slider
        if mode =='slider':
            #self.model = rbdl.loadModel("/home/nooshin/minicheetah/src/first_leg/scripts/legRBDL.urdf")
            self.model = rbdl.loadModel("/home/lenovo/catkin_ws/src/monoped/scripts/legRBDL.urdf")
        else:
            #self.model = rbdl.loadModel("/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")
            self.model = rbdl.loadModel("/home/lenovo/catkin_ws/src/monoped/scripts/leg_RBDL.urdf")
        self.q = q
        self.qdot = qdot
        self.calf_length = -0.240
        self.end_point = np.asarray([0.0, 0.0, self.calf_length])
        self.fb_dim = 3                                                          # TODO: check this
        self.point_F_dim = 1                                                     # TODO: check this
        self.qdim = self.model.q_size
        self.mass_hip = 0.63
        self.mass_thigh = 1.062
        self.mass_calf = 0.133 
        self.S = np.hstack((np.zeros((self.qdim - self.fb_dim, self.fb_dim)), np.eye(self.qdim - self.fb_dim)))
        # self.calcJc = self.calcJc(self.q)
        # self.Calch = self.Calch(self.q,self.qdot)
        # self.CalcM = self.CalcM(self.q)


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

    def vel_slider(self,q,qdot):
        vel = rbdl.CalcPointVelocity(self.model, q, qdot, self.model.GetBodyId('jump'), np.array([0.0,0.0,0.00000]))
        return vel


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

    def inv_kin(q, qdot, qddot, size, model):
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
        
    def CalcJacobian(self, model, q, bodyid, point):
        Jc = np.zeros((3, model.dof_count))
        rbdl.CalcPointJacobian(model, q, bodyid, point, Jc)
        return Jc

    def CalcBodyToBase(self, body_id, body_point_position, \
    calc_velocity = False, update_kinematics=True, index = -1, q = None, qdot = None):
        if q is not None: qq = q
        else: qq = self.q[index, :]
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, qq, \
            body_id, body_point_position, update_kinematics)
        if not calc_velocity: return pose
        else:
            if qdot is not None: qqdot = qdot
            else: qqdot = self.qdot[index, :]
            vel = rbdl.CalcPointVelocity(self.model, qq, \
            qqdot, body_id, body_point_position, update_kinematics)
            return pose, vel
    def get_com(self, calc_velocity=False, calc_angular_momentum=False, \
                update=True, index=-1, body_part='slider', q=None, qdot=None):
        #        TODO: error for the whole body (body_part = 'hf') and calc_velocity = True

        com = np.zeros(3)
        if calc_velocity:
            com_vel = np.zeros(3)
        else:
            com_vel = None
        if calc_angular_momentum:
            angular_momentum = np.zeros(3)
        else:
            angular_momentum = None

        if q is not None:
            qq = q
        else:
            qq = self.q[index, :]
        if qdot is not None:
            qqdot = qdot
        else:
            qqdot = self.qdot[index, :]

        if body_part == 'slider':
            rbdl.CalcCenterOfMass(self.model, qq, \
                                  qqdot, com, com_vel, angular_momentum, update)

            if calc_velocity and calc_angular_momentum:
                return com, com_vel, angular_momentum
            elif calc_velocity and not calc_angular_momentum:
                return com, com_vel
            else:
                return com
        else:
            com, vel = self.__calculateBodyCOM(qq, \
                                               qqdot, calc_velocity, update, body_part)
            if calc_velocity:
                return com, vel
            else:
                return com

    def __calculateBodyCOM(self, q, dq, calc_velocity, update, body_part):
        if body_part == 'h':
            p0 = self.CalcBodyToBase(self.model.GetBodyId('jump'),
                                     np.array([0.03, 0, 0.0]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p1 = self.CalcBodyToBase(self.model.GetBodyId('hip'),
                                     np.array([0.03, 0, 0.0]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p2 = self.CalcBodyToBase(self.model.GetBodyId('thigh'),
                                     np.array([0.0, 0.06, -0.02]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p3 = self.CalcBodyToBase(self.model.GetBodyId('calf'),
                                     np.array([0., 0.01, (1 / 2) * self.calf_length]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)

            if not calc_velocity:
                com = (2*p0+self.mass_hip * p1 + self.mass_thigh * p2 + self.mass_calf * p3) / \
                      (2+self.mass_hip + self.mass_thigh + self.mass_calf)
                vel = None
            else:
                com = (2*p0[0]+self.mass_hip * p1[0] + self.mass_thigh * p2[0] + self.mass_calf * p3[0]) / \
                      (2+self.mass_hip + self.mass_thigh + self.mass_calf)
                vel = (2*p0[1]+self.mass_hip * p1[1] + self.mass_thigh * p2[1] + self.mass_calf * p3[1]) / \
                      (2+self.mass_hip + self.mass_thigh + self.mass_calf)


        return com, vel
    

    def computeJacobianCOM(self, body_part,q):
        bis = []
        pts = []
        ms = []
        if body_part == 'slider':
            bis.append(self.model.GetBodyId('jump'))
            bis.append(self.model.GetBodyId('hip'))
            bis.append(self.model.GetBodyId('thigh'))
            bis.append(self.model.GetBodyId('calf'))
            ######################## from urdf model ########################
            pts.append(np.array([0.03, 0, 0.0]))
            pts.append(np.array([0.03, 0, 0.0]))
            pts.append(np.array([0.0, 0.06, -0.02]))
            pts.append(np.array([0.0, 0.0, -0.240]))
            ms = [2,self.mass_hip, self.mass_thigh, self.mass_calf]

        else:
            print("body part should be slider")

        J = np.zeros((3, self.qdim))

        for i, bi in enumerate(bis):
            J += ms[i] * self.CalcJacobian(self.model, q, bi, pts[i])

        return J / sum(ms)

    # def calculateBodyCOM(self, q, dq, calc_velocity, update):
    #     p1 = self.CalcBodyToBase(self.model.GetBodyId('hip'), 
    #                                 np.array([0.03, 0 ,0.0]),
    #                                 update_kinematics = update,
    #                                 q = q, qdot = dq, calc_velocity = calc_velocity)
    #     p2 = self.CalcBodyToBase(self.model.GetBodyId('thigh'), 
    #                                 np.array([0.0, 0.06, -0.02]),
    #                                 update_kinematics = update,
    #                                 q = q, qdot = dq, calc_velocity = calc_velocity)
    #     p3 = self.CalcBodyToBase(self.model.GetBodyId('calf'), 
    #                                 np.array([0., 0.01, self.calf_length]),
    #                                 update_kinematics = update,
    #                                 q = q, qdot = dq, calc_velocity = calc_velocity)
        
    #     if not calc_velocity:
    #         com = (self.mass_hip*p1 + self.mass_thigh*p2 + self.mass_calf*p3)/\
    #             (self.mass_hip + self.mass_thigh + self.mass_calf)
    #         vel = None
    #     else:
    #         com = (self.mass_hip*p1[0] + self.mass_thigh*p2[0] + self.mass_calf*p3[0])/\
    #                 (self.mass_hip + self.mass_thigh + self.mass_calf)
    #         vel = (self.mass_hip*p1[1] + self.mass_thigh*p2[1] + self.mass_calf*p3[1])/\
    #                 (self.mass_hip + self.mass_thigh + self.mass_calf)
    #     return com,vel

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
        res_final=self.ForwardDynamics(qqdot.flatten(), M, h, self.S, u, Jc, p)
        return res_final

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
    
    def Liftoff_GRF(self, t, y, leg):
        if hasattr(self, 'for_refine'): u = self.u[-1, :]
        else:
            yprev = np.concatenate((self.q[-1, :], self.qdot[-1, :]))
            if np.allclose(y, yprev): u = self.u[-1, :]
            else: u = self.u0 
    #        index = self.__p0.index(leg)
        self.ComputeContactForce(y, self.__p0, u)
        if leg == 1: tt = self.tt_h
        elif leg == 2: tt = self.tt_f
        
        if t - tt < .25*self.slip_st_dur:
            return -1
        else:
            return - self.Lambda[(leg - 1)*2 + 1]
    #        if leg == 1: return t - self.tt_h - self.slip_st_dur
    #        elif leg == 2: return t - self.tt_f - self.slip_st_dur
    #        elif leg == 2: return - self.Lambda[(leg - 1)*2 + 1] - 50
    
    def __dyn(self, x, t):
        """
        .dyn  evaluates system dynamics
        """
        q = x[:self.qdim]
        qd = x[self.qdim:]
                
        self.M = self.CalcM(self.model, q)
        self.Jc = self.Jc_from_cpoints(self.model, q, self.body, self.__p0)
        self.h = self.Calch(self.model, q, qd)
        
        self.ForwardDynamics(x, self.M, self.h, self.S, self.u0, self.Jc, self.__p0) 
        
        dx = np.concatenate((qd, self.qddot.flatten()))

        return dx



    def dyn_RK4(self, t, x):
        """
        .dyn  evaluates system dynamics
        """
        return self.__dyn(x, t)
    
    def RK4(self, f):
        return lambda t, y, dt: (
                lambda dy1: (
                lambda dy2: (
                lambda dy3: (
                lambda dy4: (dy1 + 2*dy2 + 2*dy3 + dy4)/6
                )( dt * f( t + dt  , y + dy3   ) )
    	    )( dt * f( t + dt/2, y + dy2/2 ) )
    	    )( dt * f( t + dt/2, y + dy1/2 ) )
    	    )( dt * f( t       , y         ) )
    
    def interpl(self, evtFun, *args):       
        t0, t1 = self.t0, self.t0 + self.dt
        y0 = self.qqdot0forRefine.copy()
        f0 = evtFun(t0, y0, *args)
        dy = self.RK4(self.dyn_RK4)
        y1 = y0 + dy(t0, y0, np.abs(t1 - t0)).flatten()
        f1 = evtFun(t1, y1, *args)
#        print 't0,f0,t1,f1', t0,f0,t1,f1
        if np.abs(f0)<np.abs(f1): self.for_refine = True
        t = t0 - f0*(t1 - t0)/(f1 - f0)
        y = y0 + dy(t0, y0, np.abs(t - t0)).flatten()
        if hasattr(self, 'for_refine'): del self.for_refine
        return t, y.reshape(1, self.qdim*2)
    
    
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
                    
            #print("gamma:", gamma)
            aux1 = np.hstack((M, -Jc.T))
            aux2 = np.hstack((Jc, np.zeros((fdim, fdim))))
            A = np.vstack((aux1, aux2))
            B = np.vstack(((tau - h).reshape(qdim, 1), gamma.reshape(fdim, 1)))
            res = np.dot(np.linalg.inv(A), B).flatten()
            self.qddot = res[:-fdim]
            self.SetGRF(cpoints,  res[-fdim:])                              
            
 #	    print("=======================================")
#           print("result:", res)
            
#            print "======================================="            
#            print 'p, lambda:', self.__p[-1], self.Lambda
#            print "======================================="
        
        return res
    

    
        

#TODO:Example:
# q = np.zeros(4)
# qdot = np.zeros(4)
# robot = ROBOT(q, qdot, mode='slider')

