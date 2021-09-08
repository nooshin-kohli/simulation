import numpy as np
import sys

sys.path.append("/home/nooshin/projects/rbdl/build/python")
import rbdl

model = rbdl.loadModel("/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")


class ROBOT():
    def __init__(self, q, qdot):
        self.q = q
        self.qdot = qdot
        self.calf_length = -0.21148
        self.end_point = np.asarray([0.0, 0.0, self.calf_length])

    def calcJc(self, q):
        jc = np.zeros((3, model.q_size))
        rbdl.CalcPointJacobian(model, q, model.GetBodyId('calf'), self.end_point, jc)
        return jc

    def velocity_end(self, q, qdot):
        vel = rbdl.CalcPointVelocity(model, q, qdot, model.GetBodyId('calf'), self.end_point)
        return vel

    def pose_end(self, q):
        pose = rbdl.CalcBodyToBaseCoordinates(model, q, model.GetBodyId('calf'), self.end_point)
        return pose

# q = np.zeros(model.q_size)
# q[0]=0.1
# q[1]=0.2
# q[2]=0.2
# qdot = np.zeros(model.q_size)
# r = ROBOT(q, qdot)
#
# print(r.calcJc(q))
