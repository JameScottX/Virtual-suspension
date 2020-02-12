
#   Quadruped robots standing balance test file
#   Created in 2020 1/12
#   Vision = 1.0
#   Author Junwen Cui

import numpy as np

global error_last, error_last2,\
        error_last_,error_last_2

#attitude error
error_last = [[0 for j in range(3)] for i in range(4)]
error_last2 = [[0 for j in range(3)] for i in range(4)]
#leg position error
error_last_  = [0 for j in range(3)]
error_last_2  = [0 for j in range(3)]


exp_state = [0,0.220,0,0,0.,0]


def balance_keeping(robot,force_func,foot_force_func,force):

    global error_last, error_last2,\
        error_last_,error_last_2

    now_state = [robot.gps_pos_now.x, robot.gps_pos_now.y, robot.gps_pos_now.z, robot.attitude.roll, robot.attitude.yaw, robot.attitude.pitch]
    error_last,error_last2,force=force_func(now_state, exp_state, error_last,error_last2 ,robot.leg_loc ,robot.mg_N ,
                                      robot.touchstate,[9000,7000] ,[120000,140000])

    now_pos = robot.leg_pos_now[0]

    error_last_, f_force = foot_force_func(now_pos,[0,-.100,0],error_last_,300,6000)
    force[0] = -np.array(f_force)

    now_pos = robot.leg_pos_now[3]
    error_last_2, f_force = foot_force_func(now_pos,[0,-.100,0],error_last_2,300,6000) 
    force[3] = -np.array(f_force)

    robot.forceset(force,-1)


