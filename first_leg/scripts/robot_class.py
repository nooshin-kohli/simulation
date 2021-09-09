import numpy as np
import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/projects/rbdl/build/python'
sys.path.append(dir)
import rbdl


class ROBOT():
    def __init__(self, q, qdot, urdf_path):     # TODO: path to leg_RBDL.urdf for robot without slider
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
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, q, self.model.GetBodyId('calf'), self.end_point)
        return pose

# example: 
# q = np.zeros(3)
# q[0] = 0.1
# q[1] = 0.2
# q[2] = 0.2
# qdot = np.zeros(3)
# r = ROBOT(q, qdot, "/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")
#
# print(r.calcJc(q))
