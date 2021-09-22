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
        self.calf_length = -0.21148
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


q = np.zeros(4)
qdot = np.zeros(4)
r = ROBOT(q, qdot, "/home/nooshin/minicheetah/src/first_leg/scripts/legRBDL.urdf")
print(r.pose_end(q))


