'''
    Author: Nooshin Kohli
'''
import sys
import numpy as np
import rospy
import rbdl


class ROBOT(object):
    def __init__(self, q, qdot, p, u, robot_name, model, param= None, terrian = None):

        # model is urdf path
        self.model = rbdl.loadModel(model)
        self.qdim = self.model.q_size
        self.fb_dim = 0  # degree of freedom of the floating base
        self.point_F_dim = 2  # number of forces acting on the contact point of foot?? It was 2
        self.name = robot_name

        self.param = param
        self.terrian = terrian
        self.bodies = ['hip', 'thigh', 'calf']
        self.joints = ['hip_joint', 'thigh_joint', 'calf_joint']
        # define mass of every link
        self.mass_hip = 0.63
        self.mass_thigh = 1.062
        self.mass_calf = 0.133

        # define length of links
        self.len_hip = 0.06
        self.len_thigh = 0.2131
        self.len_calf = 0.21148

        self.l_end = self.len_calf

        # states
        if q.any():
            self.q = np.array(q)
        else:
            self.q = np.zeros((1, self.qdim))
        # states
        if qdot.any():
            self.qdot = np.array(qdot)
        else:
            self.qdot = np.zeros((1, self.qdim))
        self.__p = list(p)  # contact feet
        if u.any():
            self.u = np.array(u)  # joint inputs
        else:
            self.u = np.zeros((1, self.qdim - self.fb_dim))
        self.cforce = []

        self.Jc = self.Jc_from_cpoints(self.model, self.q[-1, :], self.body, self.__p[-1])

        self.M = self.CalcM(self.model, self.q[-1, :])
        self.h = self.Calch(self.model, self.q[-1, :], self.qdot[-1, :])
        self.S = np.hstack((np.zeros((self.qdim - self.fb_dim, self.fb_dim)), np.eye(self.qdim - self.fb_dim)))

    def ComputeContactForce(self, qqdot, p, u):

        q = qqdot[:self.qdim]
        qdot = qqdot[self.qdim:]

        Jc = self.Jc_from_cpoints(self.model, q, self.body, p)

        M = self.CalcM(self.model, q)

        h = self.Calch(self.model, q, qdot)

        self.ForwardDynamics(qqdot.flatten(), M, h, self.S, u, Jc, p)

        return None

    def Calch(self, model, q, qdot):
        h = np.zeros(model.q_size)
        rbdl.InverseDynamics(model, q, qdot, np.zeros(self.model.qdot_size), h)
        return h

    def calcM(self, model, q):
        M = np.zeros((model.q_size, model.q_size))
        rbdl.CompositeRigidBodyAlgorithim(model, q, M, True)
        return M

    def CalcJacobian(self, model, q, bodyid, point):
        Jc = np.zeros((3, model.dof_count))
        rbdl.CalcPointJacobian(model, q, bodyid, point, Jc)
        return Jc

    def Jc_from_cpoints(self, model, q, body, cpoints):

        Jc = np.array([])

        ftip_pose = np.array([body.l_end, 0., 0.])

        if 1 in cpoints:
            Jc_ = self.CalcJacobian(model, q, self.model.GetBodyId('calf'), ftip_pose)
            Jc = np.append(Jc, Jc_[:2, :])
        return Jc.reshape(np.size(Jc) / model.dof_count, model.dof_count)

    def CalcGamma(self, cp, q, qdot):
        self.cbody_id = []
        if 1 in cp:
            for i in range(self.point_F_dim):
                self.cbody_id.append(self.model.GetBodyId('calf'))

    def ForwardDynamics(self, x, M, h, S, tau, Jc, cpoints):
        fdim = np.shape(Jc)[0]
        qdim = self.qdim
        q = x[:qdim]
        qdot = x[qdim:]

        if fdim == 0:

            self.qddot = np.dot(np.linalg.inv(M), np.dot(S.T, self.u0) - h).flatten()
            self.Lambda = np.zeros(fdim) * np.nan
        else:

            if np.nonzero(qdot)[0].any() and Jc.any():
                gamma = self.CalcGamma(cpoints, q, qdot)
            else:
                gamma = - np.dot(np.zeros_like(Jc), qdot)

            aux1 = np.hstack((M, -Jc.T))
            aux2 = np.hstack((Jc, np.zeros((fdim, fdim))))
            A = np.vstack((aux1, aux2))

            B = np.vstack(((np.dot(S.T, tau) - h).reshape(qdim, 1), \
                           gamma.reshape(fdim, 1)))

            res = np.dot(np.linalg.inv(A), B).flatten()

            self.qddot = res[:-fdim]

            self.SetGRF(cpoints, res[-fdim:])

        return None

    def CalcAcceleration(self, q, qdot, qddot, body_id, body_point):
        body_a = rbdl.CalcPointAcceleration(self.model, q, qdot, qddot, body_id, body_point)
        return body_a

    def CalcBodyToBase(self, body_id, body_point_position, calc_velocity=False, update_kinematics=True, index=-1,
                       q=None, qdot=None):
        if q is not None:
            qq = q
        else:
            qq = self.q[index, :]
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, qq, body_id, body_point_position, update_kinematics)
        if not calc_velocity:
            return pose
        else:
            if qdot is not None:
                qqdot = qdot
            else:
                qqdot = self.qdot[index, :]
            vel = rbdl.CalcPointVelocity(self.model, qq, qqdot, body_id, body_point_position, update_kinematics)
            return pose, vel

    def SetGRF(self, p, values):
        #        print 'yes', p
        last = 0
        if 1 in p:
            p_1 = last
            last += self.point_F_dim
        if 2 in p:
            p_2 = last
            last += self.point_F_dim
        self.Lambda = np.zeros(2 * self.point_F_dim) * np.nan
        if 1 in p:
            self.Lambda[:self.point_F_dim] = values[p_1:p_1 + self.point_F_dim]
        if 2 in p:
                self.Lambda[self.point_F_dim:2 * self.point_F_dim] = values[p_2:p_2 + self.point_F_dim]
        return None

    def set_input(self, tau):
        if len(tau) == self.qdim:
            self.u0 = np.dot(self.S, tau)
        else:
            self.u0 = tau
            #        print self.u0
        return None

    def get_com(self, calc_velocity=False, calc_angular_momentum=False, update=True, index=-1, body_part='leg', q=None,
                qdot=None):

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

        if body_part == 'leg':
            rbdl.CalcCenterOfMass(self.model, qq, qqdot, com, com_vel, angular_momentum, update)

            if calc_velocity and calc_angular_momentum:
                return com, com_vel, angular_momentum
            elif calc_velocity and not calc_angular_momentum:
                return com, com_vel
            else:
                return com
        else:
            com, vel = self.__calculateBodyCOM(qq, qqdot, calc_velocity, update, body_part)
            if calc_velocity:
                return com, vel
            else:
                return com

    def __calculateBodyCOM(self, q, dq, calc_velocity, update, body_part):

        p1 = self.CalcBodyToBase(self.model.GetBodyId('hip'),
                                 np.array([0., 0., -1*(self.len_hip)]),
                                 update_kinematics=update,
                                 q=q, qdot=dq, calc_velocity=calc_velocity)
        p2 = self.CalcBodyToBase(self.model.GetBodyId('thigh'),
                                 np.array([0., 0., -1*(self.len_thigh)]),
                                 update_kinematics=update,
                                 q=q, qdot=dq, calc_velocity=calc_velocity)
        p3 = self.CalcBodyToBase(self.model.GetBodyId('calf'),
                                 np.array([0., 0., -1*(self.len_calf)]),
                                 update_kinematics=update,
                                 q=q, qdot=dq, calc_velocity=calc_velocity)

        if not calc_velocity:
            com = (self.mass_hip * p1 + self.mass_thigh * p2 + self.mass_calf * p3) / \
                  (self.mass_hip + self.mass_thigh + self.mass_calf)
            vel = None
        else:
            com = (self.mass_hip * p1[0] + self.mass_thigh * p2[0] + self.mass_calf * p3[0]) / \
                  (self.mass_hip + self.mass_thigh + self.mass_calf)
            vel = (self.mass_hip * p1[1] + self.mass_thigh * p2[1] + self.mass_calf * p3[1]) / \
                  (self.mass_hip + self.mass_thigh + self.mass_calf)

        return com, vel

    def CalcBodyToBase(self, body_id, body_point_position, calc_velocity=False, update_kinematics=True, index=-1,
                       q=None, qdot=None):
        if q is not None:
            qq = q
        else:
            qq = self.q[index, :]
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, qq, body_id, body_point_position, update_kinematics)
        if not calc_velocity:
            return pose
        else:
            if qdot is not None:
                qqdot = qdot
            else:
                qqdot = self.qdot[index, :]
            vel = rbdl.CalcPointVelocity(self.model, qq, qqdot, body_id, body_point_position, update_kinematics)
            return pose, vel

    def computeJacobianCOM(self, body_part):
        bis = []
        pts = []
        ms = []
        if body_part == 'leg':
            bis.append(self.model.GetBodyId('hip'))
            bis.append(self.model.GetBodyId('thigh'))
            bis.append(self.model.GetBodyId('calf'))
            pts.append(np.array([self.len_hip, 0., 0.]))
            pts.append(np.array([self.len_thigh, 0., 0.]))
            pts.append(np.array([self.len_calf, 0., 0.]))
            ms = [self.mass_hip, self.mass_thigh, self.mass_calf]

        J = np.zeros((3, self.qdim))

        for i, bi in enumerate(bis):
            J += ms[i] * self.CalcJacobian(model, self.q[-1, :], bi, pts[i])

        return J[:2, :] / sum(ms)

    def computeFootState(self, body_part, calc_velocity=False, update_kinematics=True, index=-1, q=None, qdot=None):

        point = np.array([self.l_end, 0., 0.])
        if body_part == 'leg':
            body_id = self.model.GetBodyId('calf')

        return self.CalcBodyToBase(body_id, point, calc_velocity=calc_velocity, update_kinematics=update_kinematics,
                                   index=index, q=q, qdot=q)

    def getContactFeet(self, total=False):
        if not total:
            return self.__p[-1]
        else:
            return self.__p

    def UpdateQdotCollision(self, q, qdot, p0, W=None):
        J = self.Jc_from_cpoints(self.model, q, self.body, p0)
        if W is None: W = self.CalcM(self.model, q)
        invW = np.linalg.inv(W)
        aux1 = np.dot(invW, J.T)
        aux2 = np.linalg.inv(np.dot(J, np.dot(invW, J.T)))
        invJ = np.dot(aux1, aux2)
        qdot_after = np.dot(np.eye(np.size(qdot)) - np.dot(invJ, J), qdot)
        return qdot_after
    def Touchdown(self, t, x, leg):
        """
                should return a positive value if leg penetrated the ground
                """
        q = x[:self.qdim]
        point = np.array([self.body.l_end, 0., 0.])

        if leg == 1:
            body_id = self.model.GetBodyId('calf')

        pose = self.CalcBodyToBase(body_id, point, q=q)
        return - (pose[1] - self.TerrainHeight(pose[0]))

    def temp(self, i):

        self.dy = self.RK4(self.dyn_RK4)

        import matplotlib.pyplot as plt

        def create(q, qdot):
            return np.concatenate((q, qdot)).reshape(1, self.qdim * 2)

        q = self.q[i, :]
        qdot = self.qdot[i, :]

        y = create(q, qdot)

        plt.plot(self.Lambda, '--*')

        self.dy(self.t[i], y.flatten(), self.dt)

        plt.plot(self.Lambda, '-o')
