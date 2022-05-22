#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Nooshin Kohli
"""

import numpy as np
import copy
from leg_robotclass import ROBOT
from scipy.optimize import root
from leg_controlclass import leg_controlclass, Control

#==============================================================================
# swing control
#==============================================================================
def ctrl_swing(robot, body_part):
    kp = np.diag([2500,2500])
    kd = np.diag([250,250])
#    desired_param = t, tt, tl, ta, xt, xl, x_foot, dx0

# location and velocity of body COM
    r, dr = robot.get_com(body_part = body_part, calc_velocity = True)
    
    
    x_des, dx_des = computeDesiredFootState(robot, body_part, r, dr)
#    print 'x_des', x_des
    x, dx = robot.computeFootState(body_part, calc_velocity = True)
    e = x_des - x[:2]
    
    de = dx_des - dx[:2]
#    print body_part, e
#    print 'de', de
    F = np.dot(kp, e) + np.dot(kd, de)
    J23 = robot.computeJacobian23(body_part = body_part)
    #return u[0] , u[1] if bodypart is h
    #return u[3] , u[4] id bodypart is f
    return np.dot(J23.T, F)
    
    
#==============================================================================
#  stance control
#============================================================================== 
def ctrl_stance(robot, body_part):

    
    t = robot.t[-1][0] + robot.dt
    if body_part == 'slider': 
        tt = robot.tt_h
        xt = robot.xt_h[0]
        xl_expected = robot.foot_pose_h + robot.slip_st_xl
        mg = (robot.param.m_hip + robot.param.m_thigh + robot.param.m_calf)*robot.param.g0
    
    tl = tt + robot.slip_st_dur

    tau = (t - tt)/(tl - tt) 
    
#    h, dh = computeYShift(t, tt, tl, robot.slip_yt, robot.slip_yl)
    r, dr = robot.get_com(body_part=body_part, calc_velocity=True)
    
    s = (r[0] - xt)/(xl_expected - xt)
    
#    print 'tau: ', tau
#    print 's: ', s
    
    
    if grf_flag_time:   
        fx = mg*robot.slip_fun_grf_x_t(tau)
        fy = mg*robot.slip_fun_grf_y_t(tau)
    else:
        fx = mg*robot.slip_fun_grf_x_s(s)
        fy = mg*robot.slip_fun_grf_y_s(s)
   
    
#    if body_part == 'f':
#        print tau, fx
    F = np.array([fx, fy]).flatten()
    J23 = robot.computeJacobian23(body_part = body_part)
    tau = np.dot(J23.T, F)
            
    return -tau.flatten()
    

def ctrl_stance_IK(robot):  
    
    t = robot.t[-1][0] + robot.dt
    
    tt_h = robot.tt_h
    xt_h = robot.xt_h[0]
    xl_expected_h = robot.foot_pose_h + robot.slip_st_xl

   
    tt_f = robot.tt_f
    xt_f = robot.xt_f[0]
    xl_expected_f = robot.foot_pose_f + robot.slip_st_xl

    tl_h = tt_h + robot.slip_st_dur
    tl_f = tt_f + robot.slip_st_dur

    tau_h = (t - tt_h)/(tl_h - tt_h) 
    tau_f = (t - tt_f)/(tl_f - tt_f) 
    
    tau = np.array([tau_h,tau_f])


    r_h, dr_h = robot.get_com(body_part='h', calc_velocity=True)
    r_f, dr_f = robot.get_com(body_part='f', calc_velocity=True)
    
    s_h = (r_h[0] - xt_h)/(xl_expected_h - xt_h)
    s_f = (r_f[0] - xt_f)/(xl_expected_f - xt_f)
    
    s = np.array([s_h,s_f]).reshape(2)


    if grf_flag_time:
        qdd_ref = compute_qddot_des(robot,tau=tau)
    else:
        qdd_ref = compute_qddot_des(robot,s=s)
    


    compute_qdot_des(robot,qdd_ref)
    compute_q_des(robot)
    
    
    control = Centauro_ControlClass(robot)
#    q_0 = np.zeros(8)
    
    
    
    torque = control.InvDyn_qr(robot.q_des[-1],robot.qdot_des[-1],qdd_ref)
    

#    control = Control(robot)
#    torque = control.torque(qdd_ref)
    
#    print('torques', torque)
    
    
    return torque.flatten()


def get_xdd_ref(xdd_des,delta_xd,delta_x,kp=4.,kd=0.4):
    
    
    if xdd_des.shape == delta_xd.shape == delta_x.shape:
        return xdd_des + kd * delta_xd + kp * delta_x
    else:
        raise Exception('matrices have different shapes')



def compute_xddref(robot,tau=None,s=None):
    
#    print('s = ' , s ,'tau = ',tau )

    xdd_des = np.array([0,0,0,0]).reshape(4,1) #must get from SLIP
    
    

    com_h , com_vel_h = robot.get_com(body_part='h', calc_velocity=True)
    com_f , com_vel_f = robot.get_com(body_part='f', calc_velocity=True)
    
    x_actual_h = np.array(com_h[:2]).reshape(1,2)
    xd_actual_h = np.array(com_vel_h[:2]).reshape(1,2)

    
    x_actual_f = np.array(com_f[:2]).reshape(1,2)
    xd_actual_f = np.array(com_vel_f[:2]).reshape(1,2)
    
    x_actual = np.concatenate((x_actual_h,x_actual_f)).reshape(4,1)
    xd_actual = np.concatenate((xd_actual_h,xd_actual_f)).reshape(4,1)


    
    if grf_flag_time:
        xd_des_h = np.array([robot.slip_fun_dx_t(tau[0]),robot.slip_fun_dy_t(tau[0])])
        x_des_h = np.array([robot.slip_fun_x_t(tau[0]),robot.slip_fun_y_t(tau[0])])
        xd_des_f = np.array([robot.slip_fun_dx_t(tau[1]),robot.slip_fun_dy_t(tau[1])])
        x_des_f = np.array([robot.slip_fun_x_t(tau[1]),robot.slip_fun_y_t(tau[1])])
    else:
        xd_des_h = np.array([robot.slip_fun_dx_s(s[0]),robot.slip_fun_dy_s(s[0])])
        x_des_h = np.array([robot.slip_fun_x_t(s[0]),robot.slip_fun_y_t(s[0])])
        xd_des_f = np.array([robot.slip_fun_dx_s(s[1]),robot.slip_fun_dy_s(s[1])])
        x_des_f = np.array([robot.slip_fun_x_t(s[1]),robot.slip_fun_y_t(s[1])])
    
#    xd_des = np.concatenate((xd_des_h,xd_des_f)).reshape(4,1)
    xd_des = np.array([0,0,0,0]).reshape(4,1)
#    x_des = np.concatenate((x_des_h,x_des_f)).reshape(4,1)
    x_des = x_actual + np.array([0.1,0,0.1,0]).reshape(4,1)

    
    xdd_ref = get_xdd_ref(xdd_des,(xd_des - xd_actual) , (x_des - x_actual))
#    delta_xd = np.array([0,0,0,0]).reshape(4,1)
#    deltax = np.array([0,0,0,0]).reshape(4,1)
#    xdd_ref = get_xdd_ref(xdd_des,delta_xd , deltax)
    
#    print('xddref_shape is',xdd_ref.shape, xdd_ref)
    return xdd_ref
        

def compute_qddot_des(robot,tau=None,s=None):

    
    if 'tau' in globals() and grf_flag_time:
        raise Exception('tau is required')
    if 's' in globals() and not grf_flag_time:
        raise Exception('s is required')
    
    if grf_flag_time:
        xdd_ref = compute_xddref(robot,tau=tau)
    else:
        xdd_ref = compute_xddref(robot,s=s)
        
#    print('xddref shape' , xdd_ref.shape)
    
    
    jdqd_h,jdqd_f = robot.CalcJgdotQdot()
    
    jdqd = np.vstack((jdqd_h,jdqd_f)).reshape(4,1) 


    jGh = robot.computeJacobianCOM('h')
    jGf = robot.computeJacobianCOM('f')
    
    jG = np.vstack((jGh,jGf)).reshape(4,8) 
    
    qdd_ref = np.matmul(np.linalg.pinv(jG),(xdd_ref-jdqd))

    
#    print(qdd_ref.shape,'qdd_ref ',qdd_ref.shape,'jdqd shape',jdqd.shape)
    return qdd_ref


def compute_qdot_des(robot,qddot_des):
    qdot_des = dt*qddot_des + robot.qdot_des[-1]
    robot.qdot_des.append(qdot_des)
    return qdot_des
    
def compute_q_des(robot):
    q_des = robot.qdot_des[-1] * dt + robot.q_des[-1]
    robot.q_des.append(q_des)
    return q_des
    
    
    
    
    
    
    
    
#==============================================================================
# state_feedback control
#==============================================================================
def ctrl_state_feedbacks(robot):
    
    t = robot.t[-1][0] + robot.dt

    kp, kd, kv = 20000, 2000, 1000*0
    kthp, kthd =  -5000*0, -500*0
    kthp_dist, kthd_dist = 300*0, 30*0
    
    th_des, dth_des = 0, 0
    
    p = robot.getContactFeet() # stance legs
    
    J = np.zeros((len(p)*2 + 1, robot.qdim))
    F_com = np.zeros((len(p)*2 + 1, 1))
#    J = np.zeros((5, robot.qdim))
#    F_com = np.zeros((5, 1))

    r_h, dr_h = robot.get_com(body_part='h', calc_velocity=True)
    r_f, dr_f = robot.get_com(body_part='f', calc_velocity=True)
    
    d = r_f - r_h
    ddot = dr_f - dr_h
    
    J_th = computeJacobianTh(robot)
    J[-1, 5] = J_th[5] # always actuate torso
    
    th = np.arctan2(d[1], d[0])
    dth = np.dot(J_th, robot.q[-1, :])
#    print np.rad2deg(th), np.rad2deg(dth)
       
    d_norm = np.linalg.norm(d)
    dd_norm = np.dot(d, ddot)/d_norm

    # for the pitch
    F_com[-1] = kthp*(th_des - th) + kthd*(dth_des - dth) + \
                kthp_dist*(robot.slip_shift - d_norm) + kthd_dist*(- dd_norm)
                
#    F_com[-1] = - F_com[-1]
    
    if 1 in p: 
        tt = robot.tt_h
        tl = tt + robot.slip_st_dur
        xt = robot.xt_h[0]
        xl_expected = robot.foot_pose_h + robot.slip_st_xl
        
        r, dr = r_h, dr_h
        
        ### kind of constant height feedback
#        h, dh = computeYShift(t, tt, tl, robot.slip_yt, robot.slip_yl)

        ### exact slip feedback
        tau = (t - tt)/(tl - tt)
        s = (r[0] - xt)/(xl_expected - xt)
        
#        print 'h', tau, s
        
        if grf_flag_time:
            h, dh = robot.slip_fun_y_t(tau), robot.slip_fun_dy_t(tau)  
            xx, dxx = robot.slip_fun_x_t(tau) + xt, robot.slip_fun_dx_t(tau)
        else:
            h, dh = robot.slip_fun_y_s(s), robot.slip_fun_dy_s(s)  
            xx, dxx = robot.slip_fun_x_s(s) + xt, robot.slip_fun_dx_s(s)
        
        
        J[-1, [3, 4]] = J_th[[3, 4]]
        J[:2, [3, 4]] = robot.computeJacobianCOM23(body_part = 'h')
        
        Fx_h = kv*(robot.slip_nominal_vel - dr[0]) + \
               -kp*(xx - r[0]) + -kd*(dxx - dr[0])
               
        Fy_h = kp*(h - r[1]) + kd*(dh - dr[1])
        F_com[:2, 0] = [Fx_h, Fy_h]

#        print 'h', h,  h - r[1], dh - dr[1]
        
    if 2 in p:
        indi, indf = 2, 4
        if 1 not in p: indi, indf = 0, 2
        tt = robot.tt_f
        tl = tt + robot.slip_st_dur
        xt = robot.xt_f[0]
        xl_expected = robot.foot_pose_f + robot.slip_st_xl

        r, dr = r_f, dr_f
        
        ### kind of constant height feedback
#        h, dh = computeYShift(t, tt, tl, robot.slip_yt, robot.slip_yl)

        ### exact slip feedback
        tau = (t - tt)/(tl - tt)
        s = (r[0] - xt)/(xl_expected - xt)
        
        if grf_flag_time:
            h, dh = robot.slip_fun_y_t(tau), robot.slip_fun_dy_t(tau)  
            xx, dxx = robot.slip_fun_x_t(tau) + xt, robot.slip_fun_dx_t(tau)
        else:
            h, dh = robot.slip_fun_y_s(s), robot.slip_fun_dy_s(s)  
            xx, dxx = robot.slip_fun_x_s(s) + xt, robot.slip_fun_dx_s(s)
        
        
        J[-1, [6, 7]] = J_th[[6, 7]]
        J[indi:indf, [6, 7]] = robot.computeJacobianCOM23(body_part = 'f')
        
        Fx_f = kv*(robot.slip_nominal_vel - dr[0]) +\
               -5.5*kp*(xx - r[0]) - 5.5*kd*(dxx - dr[0])
        
        Fy_f = -5.5*kp*(h - r[1]) - 5.5*kd*(dh - dr[1])
        F_com[indi:indf, 0] = [Fx_f, Fy_f]

#        print '>>>> f >>>>', h - r[1], dh - dr[1]


#    print F_com.flatten()
        
#        print '\n\n new: \n', -np.dot(J.T, F_com)[3:].flatten()
    
    u_feedback = np.dot(J.T, F_com)[3:].flatten()
    
    u_feedback[2] = ctrl_torso(robot, robot.slip_shift)
#    u_feedback[2] = 0
    
#    print u_feedback
    
    return u_feedback









    
#==============================================================================
# common function
#==============================================================================
def computeDesiredFootState(robot, body_part, x_now, dx_now):
#    t, tt, tl, ta, xt, xl, x_foot= param
    
    t = robot.t[-1][0] + robot.dt

    if body_part == 'h': 
        tl = robot.tl_h
        xxi = robot.foot_pose_h - robot.xl_h[0]
    elif body_part == 'f': 
        tl = robot.tl_f
        xxi = robot.foot_pose_f  - robot.xl_f[0]
    
    tt = tl + robot.slip_sw_dur
    
#    if body_part == 'h' and ((ta + tt) - t) < 0.05: tt = t 
    
#    ti = tl - ta # previous liftoff approximation
#    tf = tt # next touchdown

    xxf = robot.slip_sw_xt
    
#    xxi = x_foot - xl[0] # previous liftoff pose relative to current state
#    xxf = x_foot - xt[0] # next touchdown relative to current state
    
#    yyi = xl[1]
#    yyf = xt[1]

#    print body_part, '>>>', (t - tl)/(tt -tl)
        
    # get the swing feet pose and ...
    r, dr, ddr = getTraj(t, tl, tt, xxi, xxf)
    h, dh = computeYShift(t, tl, tt, robot.slip_yt, robot.slip_yl)
    
#    print 'h, dh', (x_now[1] - h), (dx_now[1] - dh)
#    print 'r', r

        
    # siwng foot global position
    x_swing = r.flatten() 
    x_swing[0] += x_now[0]
    x_swing[1] += (x_now[1] - h)
    
#    if body_part == 'h' and t > ta and (tt - t) < 0.05: 
#        print 'hind foot desired traj adjusted !!!!'
#        x_swing[1] = 0

    # siwng foot global velocity
    dx_swing = dr.flatten() 
    dx_swing[0] += dx_now[0]
    dx_swing[1] += (dx_now[1] - dh)
    return x_swing, dx_swing
    
    
def computeYShift(t, ti, tf, hl, ht):
    tau = (t - ti)/(tf - ti)
    return hl*(1 - tau) + ht*tau, ht - hl


def ctrl_torso(robot, shift):
    
    r1, dr1 = robot.get_com(body_part = 'h', calc_velocity = True)
    r2, dr2 = robot.get_com(body_part = 'f', calc_velocity = True)
    
    d = r2 - r1
    ddot = dr2 - dr1
    
    d_norm = np.linalg.norm(d)
    dd_norm = np.dot(d, ddot)/d_norm

    tau = 0

    tau += 500*(robot.slip_shift - d_norm) + 50*(- dd_norm)
    
#    tau += 500*(np.abs(d[1])) + 50*(-np.abs(ddot[1]))
        
    return tau


def computeJacobianTh(robot):
    r_h, dr_h = robot.get_com(body_part='h', calc_velocity=True)
    r_f, dr_f = robot.get_com(body_part='f', calc_velocity=True)
    xh, yh = r_h[:2]
    dxh, dyh = dr_h[:2]
    xf, yf = r_f[:2]
    dxf, dyf = dr_f[:2]
    
    J_h = robot.computeJacobianCOM('h')
    J_f = robot.computeJacobianCOM('f')
    
    mup = (xf - xh)*(J_f[1, :] - J_h[1, :]) - (yf - yh)*(J_f[0, :] - J_h[0, :])
    mu = 1/((xf - xh)**2 + (yf - yh)**2)
    J = mu*mup
    return J
    

#==============================================================================
# Initial configuration (IK)
#==============================================================================
def computeInitialConfig(robot, des_config, nlayer):
    tt, tl, ta, x0_h, xt_h, xl_h, xa_h, x_foot_h, dx0_h,\
                x0_f, xt_f, xl_f, xa_f, x_foot_f, dx0_f = des_config
                
    x_swing_h, dx_swing_h = computeDesiredFootState(robot, 'h', x0_h, dx0_h)
    
    x_swing_f, dx_swing_f = computeDesiredFootState(robot, 'f', x0_f, dx0_f)
    


    des = np.array([x0_h, x_swing_h, x0_f, x_swing_f, \
                    dx0_h, dx_swing_h, dx0_f, dx_swing_f]).flatten()
    
    model = ROBOT(t, q, qdot, p, u, dt, lua_file, param)
    
    if nlayer == 1:
        args = ([model, des, np, nlayer, 'first'])
        IC = np.concatenate((robot.q[0, :], robot.qdot[0, :]))
        res = root(CostIK, IC, args=args)
        out = res.x.flatten()
        
    elif nlayer == 2:
        args = ([model, des[:8], np, nlayer, 'first'])
        IC = robot.q[0, :]
        res1 = root(CostIK, IC, args=args)
        
#        print 'res1.fun', res1.fun
        
        model.q = res1.x.reshape(1, 8)
        args = ([model, des[8:], np, nlayer, 'second'])
        IC = robot.qdot[0, :]
        res2 = root(CostIK, IC, args=args)
        
#        print 'res2.fun', res2.fun
        
        out = np.concatenate((res1.x.flatten(), res2.x.flatten()))
        
#    print res.fun

    return out
    
def CostIK(x, args):

    model, des, np, nlayer, segment = args  
    
    #x0_h, x_swing_h, x0_f, x_swing_f, dx0_h, dx_swing_h, dx0_f, dx_swing_f
    
    if nlayer == 1:
        r1, r2, r3, r4, r5, r6, r7, r8, \
        r9, r10, r11, r12, r13, r14, r15, r16 = des 
    
        model.q = x[:8].reshape(1, 8)
        model.qdot = x[8:].reshape(1, 8)
        model.qdot[0, 0] = r9

    elif nlayer == 2:
        if segment == 'first':
            r1, r2, r3, r4, r5, r6, r7, r8, = des 
            model.q = x.reshape(1, 8)

        elif segment == 'second':
            r9, r10, r11, r12, r13, r14, r15, r16 = des 
            model.qdot = x.reshape(1, 8)
#            model.qdot[0, 0] = r9
#            model.qdot[0, 5] = 0
            
    xh, dxh = model.get_com(body_part = 'h', calc_velocity = True)
    xf, dxf = model.get_com(body_part = 'f', calc_velocity = True)
    
    point = np.array([model.body.l_end, 0., 0.])
    
    x_swing_h, dx_swing_h  = \
    model.CalcBodyToBase(model.body.id('b3h'), point, calc_velocity = True)
    
    x_swing_f, dx_swing_f = \
    model.CalcBodyToBase(model.body.id('b3f'), point, calc_velocity = True)
    
    
    if nlayer == 1:
        e = list()
        e.append((xh[0] - r1)**2)
        e.append((xh[1] - r2)**2)
        e.append((x_swing_h[0] - r3)**2)
        e.append((x_swing_h[1] - r4)**2)
        e.append((xf[0] - r5)**2)
        e.append((xf[1] - r6)**2)
        e.append((x_swing_f[0] - r7)**2)
        e.append((x_swing_f[1] - r8)**2)
        
        e.append((dxh[0] - r9)**2)
        e.append((dxh[1] - r10)**2)
        e.append((dx_swing_h[0] - r11)**2)
        e.append((dx_swing_h[1] - r12)**2)
        e.append((dxf[0] - r13)**2)
        e.append((dxf[1] - r14)**2)
        e.append((dx_swing_f[0] - r15)**2)
        e.append((dx_swing_f[1] - r16)**2)
        
    elif nlayer == 2 and segment == 'first':
        e = list()
        e.append((xh[0] - r1)**2)
        e.append((xh[1] - r2)**2)
        e.append((x_swing_h[0] - r3)**2)
        e.append((x_swing_h[1] - r4)**2)
        e.append((xf[0] - r5)**2)
        e.append((xf[1] - r6)**2)
        e.append((x_swing_f[0] - r7)**2)
        e.append((x_swing_f[1] - r8)**2)
        
    elif nlayer == 2 and segment == 'second':
        e = list()
        e.append((dxh[0] - r9)**2)
        e.append((dxh[1] - r10)**2)
        e.append((dx_swing_h[0] - r11)**2)
#        e.append((dx_swing_h[1] - r12)**2)
        e.append((model.qdot[0, 0] - r9)**2)
        e.append((dxf[0] - r13)**2)
        e.append((dxf[1] - r14)**2)
        e.append((dx_swing_f[0] - r15)**2)
#        e.append((dx_swing_f[1] - r16)**2)
        e.append((model.qdot[0, 5])**2)
        
    
    return np.array(e, dtype=float)

def testInitialConfig():
    tt = 0.1489017
    tl = 0.35038529
    ta = 0.49774645
    x0_h = np.array([0, .75])
    xt_h = np.array([0.44670511,  0.64124773])
    xl_h = np.array([1.0088164 ,  0.64094974])
    xa_h = np.array([1.45052468e+00,   7.47463342e-01])
    x_foot_h = 0.72742089
    
    shift = .9*(cr.param.l1h + cr.param.l1f)
    horiz_shift = np.array([shift, 0])
    x0_f, xt_f, xl_f, xa_f, x_foot_f = \
    x0_h + horiz_shift, xt_h + horiz_shift, xl_h + horiz_shift, \
    xa_h + horiz_shift, x_foot_h + horiz_shift[0]
    
    des_config = tt, tl, ta, x0_h, xt_h, xl_h, xa_h, x_foot_h, \
                             x0_f, xt_f, xl_f, xa_f, x_foot_f
    
#    print computeInitialConfig(0, des_config)
    return computeInitialConfig(cr, des_config)   
    






#
#
#p = np.linspace(0, 1, 100)   
#for i in p:
#    plt.figure('grfx')
#    plt.plot(i, cr.slip_fun_grf_x(i), '+')
#    plt.plot(i, crt.slip_fun_grf_x(i), '*')
#    
#    plt.figure('grfy')
#    plt.plot(i, cr.slip_fun_grf_y(i), '+')
#    plt.plot(i, crt.slip_fun_grf_y(i), '*')
#    
#    plt.figure('x')
#    plt.plot(i, cr.slip_fun_x(i), '+')
#    plt.plot(i, crt.slip_fun_x(i), '*')
#    
#    plt.figure('dx')
#    plt.plot(i, cr.slip_fun_dx(i), '+')
#    plt.plot(i, crt.slip_fun_dx(i), '*')
#    
#    plt.figure('y')
#    plt.plot(i, cr.slip_fun_y(i), '+')
#    plt.plot(i, crt.slip_fun_y(i), '*')
#    
#    plt.figure('dy')
#    plt.plot(i, cr.slip_fun_dy(i), '+')
#    plt.plot(i, crt.slip_fun_dy(i), '*')
#
#
#
#
def remove_zeros(array,tol=1e-5):
    for i in range(len(array)):
        if array[i] < tol:
            array[i] = 0
    return array

def compute_desired_com_pos(t,tt,tl,v0_x,y0,vl_x,yl,vl_y):
    if t<tt :
        x = v0_x * t
        y = -0.5*cr.param.g0* t ** 2 + y0
        if tt - t <= 0.02:
            print(('tt x and y approximately',x,y))
        print('a->t')
        return [x,y]
    elif t>tl :
        tau = t - tl
        x = vl_x * t
        y = -0.5*cr.param.g0* tau ** 2 + vl_y*tau + yl
        if t - tl <= 0.02:
            print(('lo x and y approximately',x,y))
        print('l->a')
        return [x,y]
    else:
        x = cr.slip_fun_x_s((t-tt)/(tl-tt)).flatten() + loaded['x'][0, 0]
        y = cr.slip_fun_y_s((t-tt)/(tl-tt)).flatten()
        print(('st',x,y))
        return [x[0],y[0]]

    
def compute_desired_com_vel(t,tt,tl,v0_x,vl_x,vl_y):
    if t<tt:
        vel_x = v0_x
        vel_y = -cr.param.g0 * t
        return [vel_x,vel_y]
    elif t>tl:
        vel_x = vl_x
        vel_y = -cr.param.g0 * (t-tl) + vl_y
        return [vel_x,vel_y]
    else:
        vel_x = cr.slip_fun_dx_s((t-tt)/(tl-tt))
        vel_y = cr.slip_fun_dy_s((t-tt)/(tl-tt))
        return [vel_x,vel_y]
    









