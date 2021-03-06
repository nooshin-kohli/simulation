import numpy as np
import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/projects/rbdl/build/python'
sys.path.append(dir)
import rbdl

urdf_path = home + '/minicheetah/src/first_leg/scripts/leg_RBDL.urdf'  # TODO: replace 'minicheetah' with your own workspace

# model = rbdl.loadModel("/home/nooshin/minicheetah/src/first_leg/scripts/leg_RBDL.urdf")
model = rbdl.loadModel(urdf_path)
end_point = np.zeros(model.q_size)
end_point[0] = 0.0
end_point[1] = 0.0
end_point[2] = -0.21148


class VP():
    def __init__(self, q, qdot):
        self.q = q
        self.qdot = qdot

        self.vel = rbdl.CalcPointVelocity(model, q, qdot, model.GetBodyId('calf'), end_point)
        self.pose = rbdl.CalcBodyToBaseCoordinates(model, q, model.GetBodyId('calf'), end_point)

    def calcJc(self, q):
        jc = np.zeros((3, model.q_size))
        rbdl.CalcPointJacobian(model, q, model.GetBodyId('calf'), end_point, jc)
        return jc


# examination

q = np.zeros(model.q_size)
q[0] = 1
q[1] = 2
q[2] = 3

qdot = np.zeros(model.q_size)
qdot[0] = 0.5
qdot[1] = 1.5
qdot[2] = 2

r = VP(q, qdot)
jc = r.calcJc(q)
print(str(jc))
