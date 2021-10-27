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

    def calcJc(self, q):
        jc = np.zeros((3, self.model.q_size))
        rbdl.CalcPointJacobian(self.model, q, self.model.GetBodyId('calf'), self.end_point, jc)
        return jc

    def velocity_end(self, q, qdot):
        vel = rbdl.CalcPointVelocity(self.model, q, qdot, self.model.GetBodyId('calf'), self.end_point)
        return vel

    def pose_end(self, q):
        pose_base = rbdl.CalcBodyToBaseCoordinates(self.model, q, self.model.GetBodyId('calf'), self.end_point)
        return pose_base

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




<<<<<<< HEAD


=======
# q[0] = -0.0791134
# q[1] = 0.489967193
# q[2] = 0.005389
>>>>>>> bf47f90740a55c5de940bbd525ce80fe534a7259
