
import numpy as np
from comm.misc.plotf import data_record,data_to_save

global dr_xyz,dr_xyz_r,dr_leg_rf_lb


dr_xyz= data_record(data_name = ['CoM-x','CoM-y','CoM-z'])
dr_xyz_r= data_record(data_name =['roll','yaw','pitch'])
dr_leg_rf_lb= data_record(data_name =['Leg-x','Leg-y','Leg-z'])


def val_rec_balance(robot,times,timestep,force):
    '''
    Balanced data logging section
    '''

    global dr_xyz,dr_xyz_r,dr_leg_rf_lb
    
    dr_xyz.save_data_times(times*timestep/1000,
                        [robot.gps_pos_now.x, robot.gps_pos_now.y, robot.gps_pos_now.z])
    dr_xyz_r.save_data_times(times*timestep/1000,
                            [robot.attitude.roll, robot.attitude.yaw, robot.attitude.pitch])                
        
    dr_leg_rf_lb.save_data_times(times*timestep/1000,[force[0][0],force[0][1],force[0][2]])   

    if times > (1000/timestep) * 8 and not dr_xyz.is_saved:
        dr_xyz.is_saved = True
        data_to_save('dr_xyz',np.array(dr_xyz.data_y).T,np.array(dr_xyz.data_x).T,'.txt')
        data_to_save('dr_xyz_r',np.array(dr_xyz_r.data_y).T,np.array(dr_xyz_r.data_x).T,'.txt')
        data_to_save('dr_leg_rf_lb',np.array(dr_leg_rf_lb.data_y).T,np.array(dr_leg_rf_lb.data_x).T,'.txt')
        print('save is ok!')
