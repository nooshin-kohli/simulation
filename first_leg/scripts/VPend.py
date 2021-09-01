import numpy as np
import sys

sys.path.append("/home/nooshin/projects/rbdl/build/python")
import rbdl

model = rbdl.loadModel("legRBDL.urdf")
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
print(r.vel)
print(r.pose[2])
