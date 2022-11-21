#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 13:02:53 2022

@author: Nooshin Kohli
"""

import control as ct 
import numpy as np
import matplotlib.pyplot as plt
A = np.array([[0,1],
              [0,0]])

B = np.array([[0,0],
             [1,-1]])

C = np.array([[1,0],
             [0,1]])

D = np.array([[0,0],
             [0,0]])

t_vec = np.linspace(0,10,100)
y_0 = [[0],
       [0]]
g = []
#print(np.shape(t_vec))

for t in t_vec:
    g.append(-10)

g = np.array(g)

F = []
for t in t_vec:
    if t<5:
        F.append(20)
    else:
        F.append(0)
#t_vec=np.reshape(t_vec,(100,1))

F = np.array(F)
# F = np.reshape(F,(100,1))
print(np.shape(F))
# g = np.reshape(g, (100,1))
U = np.array([F,g])
# U = np.flatten(U)
# U = U.flatten()
# U = U.flatten()
sys = ct.ss(A,B,C,D)
print(sys)  
y,ydot = ct.input_output_response(sys, U, t)

plt.figure()
# #plt.plot(y)
plt.plot(ydot)
plt.show()

