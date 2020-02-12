#   Virtual suspension model control method
#   Updated in 2020 1/4
#   Vision = 2.6
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

def desire_pos_cal(body_now,body_deire,corner_pos,*args):
    '''
    Calculate expected foot force based on position error
    Input:      the current body posture and the target body attitude.
                Hip joint coordinates, fixed parameters
                whether double / single separates position and pose
                One leg position is calculated each time
    Output:     a vector in x, y, z directions
    '''


    if  len(corner_pos)!=3:
        raise ValueError('corner_pos_ have the wrong length!')

    if not (len(body_now) == len(body_deire)):
        raise ValueError('body_now and body_deire must have the same length!')

    if args==None:
        raise ValueError('desire_pos_cal must has args values!')

    corner_pos_m = np.mat([[corner_pos[0]],[corner_pos[1]],[corner_pos[2]]])  #corner_pos position

    #desire part
    body_derire_xyz = np.mat([[body_deire[0]],[body_deire[1]],[body_deire[2]]])   #CoM desire position
    R_mat = np.mat(R_Matrix(body_deire[3],body_deire[4],body_deire[5]))    
    Expect_ang = R_mat * corner_pos_m

    #now part
    body_now_zyx = np.mat([[body_now[0]],[body_now[1]],[body_now[2]]])   #Body present position
    R_mat = np.mat(R_Matrix(body_now[3],body_now[4],body_now[5]))
    Now_ang = R_mat * corner_pos_m

    #print(Now_xyz)
    #print(Expect_xyz)

    dis_xyz = 1.*(body_derire_xyz - body_now_zyx)  #CoM deisre xyz vector
    dis_ang = 1.*(Expect_ang - Now_ang)            #CoM deisre roll yaw pitch angle

    if args[0] == 'double':

        dis_xyz_  = np.squeeze(np.array(dis_xyz))  
        dis_ang_  = np.squeeze(np.array(dis_ang))  
        return dis_xyz_,dis_ang_

    elif args[0] == 'single':

        dis_xyz_ang = dis_xyz + dis_ang
        dis_xyz_ang_  = np.squeeze(np.array(dis_xyz_ang)) 
        return dis_xyz_ang_
    else :
        raise ValueError('desire_pos_cal args must be double or single')



def desire_pos_cal_asse(body_now,body_deire,corner_pos_,*args):
    '''
    Calculate foot expected force position error
    Input:    current body posture and target body posture len = 6
              Hip joint coordinates, fixed parameters nX3
              ** kwargs contains
              biped, double / single
    Output:   a 4X3 array representing the x, y, and z directions of LF RF LB RB-default
              2X3 array represents the x, y, and z directions of LF RF
              1X3 array represents the x, y, and z direction of the Leg
    '''


    if args==None:
        raise ValueError('desire_pos_cal_asse must has args values!')

    size_leg = args[0]

    if type(corner_pos_) is not np.ndarray:
        corner_pos_ = np.array(corner_pos_)

    if np.shape(corner_pos_)!= (size_leg,3):
        raise ValueError('corner_pos_ have the wrong shape!')

    if args[1] == 'double':
        dis_xyz_asse, dis_ang_asse= [],[]
        for i in range(size_leg):
            corner_pos_one = corner_pos_[i]
            dis_xyz_,dis_ang_ = desire_pos_cal(body_now,body_deire,corner_pos_one,'double')
            dis_xyz_asse.append(dis_xyz_)
            dis_ang_asse.append(dis_ang_)
        return np.array(dis_xyz_asse),np.array(dis_ang_asse)

    elif args[1] == 'single':
        dis_xyz_asse = []
        for i in range(size_leg):
            corner_pos_one = corner_pos_[i]
            dis_xyz_ang = desire_pos_cal(body_now,body_deire,corner_pos_one,'single')
            dis_xyz_asse.append(dis_xyz_ang)
        return np.array(dis_xyz_asse)
    else :
        raise ValueError('desire_pos_cal_asse args must have double or single')
    


def force_out(body_now,body_deire,dis_xyz_asse_last,corner_pos_,mg_N,touchstate,K,D,**kwargs):
    '''
    Single Spring-Damper
    Calculate foot expectations
    Input:       the current body posture and the target body posture len = 6 coordinates according to the coordinate system of the body relative to the world
                 Last position error, determined by differential in external program
                 n legs hip joints nX3
                 K, D are spring coefficient and damping coefficient, here the legs are common
                 Need to keep the position of xyz coordinates
                 The force of the body to the supporting leg N and the contact state of the foot
    Output:      this time vector error nX3
                 A 4X3 array represents the expected force in the x y z direction of LF RF LB RB-default
                 or a 2X3 array representing the expected force in the x y z direction of the LF RF
                 or a 1X3 array representing the expected force in the x y z direction of the Leg
    '''


    leg_num = 4
    speed_desire = 0  
    for name, value in kwargs.items():
        if name == 'biped': leg_num = 2
        elif name == 'singled': leg_num = 1
        elif name == 'quadruped': leg_num = 4
        elif name =='speed_desire': speed_desire = value

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)

    dis_xyz_asse = desire_pos_cal_asse(body_now,body_deire,corner_pos_,leg_num,'single')

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')

    #out_ = K * dis_xyz_asse +  D * (speed_desire - (dis_xyz_asse -  np.array(dis_xyz_asse_last))) 
    out_ = kd_cal(dis_xyz_asse,speed_desire,dis_xyz_asse_last,K,D)

    #Add force based on contact
    t = 0
    for i in touchstate: 
         t +=1 if i==1 else 0
    if t ==0: t=1
    #print(t)
    mg_N_ = mg_N / t
    for i in out_:
        i[1] += mg_N_

    return   dis_xyz_asse,out_


def force_out_2(body_now,body_deire,dis_xyz_asse_last,dis_ang_asse_last,corner_pos_,mg_N,touchstate,K,D,**kwargs):
    '''
    Double Spring-Damper
    Calculate foot expectations
    Input:   the current body posture and the target body posture len = 6 coordinates according to the coordinate system of the body relative to the world
             Last position error, determined by differential in external program
             Position error after last attitude conversion
             n legs hip joints nX3
             K, D are spring coefficient and damping coefficient.Here, the general len of the leg is 2
             Need to keep the position of xyz coordinates
             The force of the body to the supporting leg N and the contact state of the foot
    Output:  the vector error nX3, the position error nX3 after the last attitude conversion
             1 4X3 array represents the expected force in the x y z direction of LF RF LB RB-default
             or 2 2X3 arrays respectively represent the expected force in the x y z direction of the LF RF
             or 2 1X3 arrays represent the expected force in the x y z direction of Leg
    '''

    leg_num = 4
    speed_desire = [0,0]  #期望速度
    for name, value in kwargs.items():
        if name == 'biped': leg_num = 2
        elif name == 'singled': leg_num = 1
        elif name == 'quadruped': leg_num = 4
        elif name =='speed_desire': speed_desire = value

    if type(dis_xyz_asse_last) is not np.ndarray:
        dis_xyz_asse_last = np.array(dis_xyz_asse_last)

    dis_xyz_asse,dis_ang_asse = desire_pos_cal_asse(body_now,body_deire,corner_pos_,leg_num,'double')

    if np.shape(dis_xyz_asse)!= np.shape(dis_xyz_asse_last):
        raise ValueError('dis_xyz_asse and  dis_xyz_asse_last must have the same shape!')
    if np.shape(dis_ang_asse)!= np.shape(dis_ang_asse_last):
        raise ValueError('dis_ang_asse and  dis_ang_asse_last must have the same shape!')

    out_ = kd_cal(dis_xyz_asse,speed_desire[0],dis_xyz_asse_last,K[0],D[0])+\
            kd_cal(dis_ang_asse,speed_desire[1],dis_ang_asse_last,K[1],D[1])

    #依据接触给腿添加重力分量
    t = 0
    for i in touchstate: 
         t +=1 if i==1 else 0
    if t ==0: t=1
    mg_N_ = mg_N / t
    for i in out_:
        i[1] += mg_N_

    return   dis_xyz_asse,dis_ang_asse,out_

















