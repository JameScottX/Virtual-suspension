#   Ordinary position or force control method
#   Updated in 2020 1/14
#   Vision = 1.0
#   Author Junwen Cui
#   Other: use numba to accelarate


import numpy as np
import numba as nb
from comm.action.basis import R_Matrix



def kd_cal(dis_,speed_des,dis_last,K,D):
    ''' 
    numba does not support internal coercion to numpy.array
    Input must be np.array
    '''


    dis_speed = dis_ -  dis_last
    out_ = K * dis_ -  D * (speed_des - dis_speed) 
    return out_


def foot_force_support(pos_now,pos_desire,dis_xyz_asse_last,mg_N,touchstate,K,D,**kwargs):
    '''
    One-leg position support spring virtual function
    Input:      the current position of the current leg and the desired position. Len = 3 The coordinates are based on the separate coordinate system of the leg.
                Last position error, determined by differential in external program
                Robot mass and foot contact status
                K, D are spring coefficient and damping coefficient, here the legs are common
                Expectation of footsteps len = 3
    '''


    speed_desire = 0  
    for name, value in kwargs.items():
        if name == 'speed_desire': speed_desire = value

    if np.shape(np.array(pos_desire))!= np.shape(np.array(pos_now)):
        raise ValueError('pos_desire and  pos_now must have the same shape!')

    dis_xyz_asse = np.array(pos_desire) - np.array(pos_now)

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)
    out_ = kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)

    #Add force based on contact
    t = 0
    for i in touchstate: 
         t +=1 if i==1 else 0
    if t ==0: t=1
    #print(t)
    mg_N_ = mg_N / t
    
    out_[1] -= mg_N_

    return dis_xyz_asse, out_



def foot_force_out(pos_now,pos_desire,dis_xyz_asse_last, K,D,**kwargs):
    '''
    One-leg position spring virtual function
    Input:      the current position of the current leg and the desired position. Len = 3 The coordinates are based on the separate coordinate system of the leg.
                Last position error, determined by differential in external program
                K, D are spring coefficient and damping coefficient, here the legs are common
                Expectation of footsteps len = 3
    '''


    speed_desire = 0  #期望速度
    for name, value in kwargs.items():
        if name == 'speed_desire': speed_desire = value

    if np.shape(np.array(pos_desire))!= np.shape(np.array(pos_now)):
        raise ValueError('pos_desire and  pos_now must have the same shape!')

    dis_xyz_asse = np.array(pos_desire) - np.array(pos_now)

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)
    out_ = kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)

    return dis_xyz_asse, out_


def foot_force_out_all(pos_now,pos_desire,dis_xyz_asse_last, K,D,**kwargs):
    '''
    Full leg position spring virtual function
    Input:      the current position of the current leg and the desired position.
                Last position error, determined by differential in external program
                K, D are spring coefficient and damping coefficient, here the legs are common
                Which legs are calculated
                Foot Expectancy nX3
    '''


    speed_desire = 0  #期望速度
    leg_use = [1,1,1,1] # --默认

    for name, value in kwargs.items():
        if name == 'speed_desire': speed_desire = value
        elif name == 'leg_use': leg_use = value

    if np.shape(np.array(pos_desire))!= np.shape(np.array(pos_now)):
        raise ValueError('pos_desire and  pos_now must have the same shape!')

    dis_xyz_asse = np.array(pos_desire) - np.array(pos_now)

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

    leg_use_ = []
    for i in leg_use:
        leg_use_.append([i,i,i])

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)

    kd_out = kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)
    out_ = np.array(leg_use_) * kd_out 

    return dis_xyz_asse, out_   #nX3



