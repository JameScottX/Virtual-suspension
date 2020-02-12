#   Biped robots standing balance test file
#   Created in 2020 1/12
#   Vision = 1.0
#   Author Junwen Cui


import numpy as np 

global error_last, error_last2
#attitude error
error_last = [[0 for j in range(3)] for i in range(1)]
error_last2 = [[0 for j in range(3)] for i in range(1)]

exp_state = [0,0.220,0,0,0.,0]

def balance_keeping(robot,force_func,force):

    global error_last, error_last2  
    now_state = [robot.gps_pos_now.x, robot.gps_pos_now.y, robot.gps_pos_now.z, robot.attitude.roll, robot.attitude.yaw, robot.attitude.pitch]
    error_last,error_last2,force=force_func(now_state, exp_state, error_last,error_last2 ,robot.leg_loc ,robot.mg_N ,
                                      robot.touchstate,[18000,16000] ,[300000,280000],singled = True)


    robot.forceset(force,-1)