#   Quadruped robots main function
#   Created in 2019 10/20
#   Vision = 1.0
#   Author Junwen Cui


import numpy as np
from controller import *
from comm.base.leg import Leg
from comm.action.basis import Pos,Attitude,Force,R_Matrix_inv,R_Matrix
from comm.action.new_vmc import force_out_2
from comm.action.raw import foot_force_out,foot_force_support



#webots model parameter name
module_name = {'lf':{'swing_motor_s': 'lf_h_1_m', 'thignleg_motor_s': 'lf_h_2_m', 'calfleg_motor_s': 'lf_h_3_m', 
                     'swing_positionsensor_s': 'lf_h_1_ps', 'thign_positionsensor_s': 'lf_h_2_ps', 'calf_positionsensor_s': 'lf_h_3_ps', 
                     'foot_touchsensor_s': 'lf_touch'}, 
               
               'lb':{'swing_motor_s': 'lb_h_1_m', 'thignleg_motor_s': 'lb_h_2_m', 'calfleg_motor_s': 'lb_h_3_m', 
                     'swing_positionsensor_s': 'lb_h_1_ps', 'thign_positionsensor_s': 'lb_h_2_ps', 'calf_positionsensor_s': 'lb_h_3_ps', 
                     'foot_touchsensor_s': 'lb_touch'}, 
               
               'rf':{'swing_motor_s': 'rf_h_1_m', 'thignleg_motor_s': 'rf_h_2_m', 'calfleg_motor_s': 'rf_h_3_m', 
                     'swing_positionsensor_s': 'rf_h_1_ps', 'thign_positionsensor_s': 'rf_h_2_ps', 'calf_positionsensor_s': 'rf_h_3_ps', 
                     'foot_touchsensor_s': 'rf_touch'}, 
               
               'rb':{'swing_motor_s': 'rb_h_1_m', 'thignleg_motor_s': 'rb_h_2_m', 'calfleg_motor_s': 'rb_h_3_m', 
                     'swing_positionsensor_s': 'rb_h_1_ps', 'thign_positionsensor_s': 'rb_h_2_ps', 'calf_positionsensor_s': 'rb_h_3_ps', 
                     'foot_touchsensor_s': 'rb_touch'},
               'imu': 'imu' ,
               'gps':'gps'
               }


#model basic unit  m 
module_info = {'virtual_height':0.026,'body_height':0.02,'swing_lenth': 0, 'thign_lenth': 0.160, 'calf_lenth': 0.160, 'body_width': 0.130, 'body_length': 0.260}

class Dog(Supervisor):

    def __init__(self):

        super(Dog, self).__init__()

        self.attitude = Attitude()  #body attitude roll pitch yaw

        self.imu = InertialUnit(module_name['imu'])
        self.imu.enable(1)  #1000Hz

        self.gps = GPS(module_name['gps'])
        self.gps.enable(1)  #1000Hz
        
        self.gps_pos_now = Pos()   #my coordinates
        self.gps_pos = Pos()       #gps raw coordinates
        self.gps_pos_bias = [0,0,0]#temporary variables

        #Leg initialization
        self.Leg_lf = Leg(module_name['lf'], module_info)
        self.Leg_lb = Leg(module_name['lb'], module_info)
        self.Leg_rf = Leg(module_name['rf'], module_info)
        self.Leg_rb = Leg(module_name['rb'], module_info)


        #**************Robot attributes**************
        self.Swing_Lenth = module_info['swing_lenth']
        self.Thign_Lenth = module_info['thign_lenth']
        self.Calf_Lenth = module_info['calf_lenth']
        self.Body_Width = module_info['body_width']
        self.Body_Length = module_info['body_length']
        self.Virtual_Height = module_info['virtual_height']
        self.Body_Height = module_info['body_height']

        #Robot ankle coordinates in body frame, LF RF LB RB respectively  
        self.leg_loc = np.array([[-0.13,self.Virtual_Height,0.065],[-0.13,self.Virtual_Height,-0.065],[0.13,self.Virtual_Height,0.065],[0.13,self.Virtual_Height,-0.065]])
        self.mg_N = 16.44 * 9.81           #Robot gravity

        self.leg_exp_len = 0.2 #Leg extension
        self.body_exp_y = self.leg_exp_len + self.Body_Height #CoM coordinates

        #**************State attributes**************
        #1 means leg is touchdown, 0 is not
        self.leg_reset = [1,1,1,1]

        #   Dynamic function
        '''
        '''


    def refresh(self):  
        '''
        Quardpueds robot refreshes state
        '''

        #Leg part 
        self.Leg_lf.refresh()
        self.Leg_lb.refresh()
        self.Leg_rf.refresh()
        self.Leg_rb.refresh()

        self.touchstate = np.array([self.Leg_lf.touchstate,self.Leg_rf.touchstate,self.Leg_lb.touchstate,self.Leg_rb.touchstate])
       
        inertial = self.imu.getRollPitchYaw()  #imu values, we correct here
        self.attitude.roll = -inertial[0]    # x-axis
        self.attitude.pitch = -inertial[1]   # z-axis
        self.attitude.yaw = inertial[2]      # y-axis

        pos = self.gps.getValues()        #gps values, , we correct here
        self.gps_pos.x = -pos[0]
        self.gps_pos.y = pos[1]
        self.gps_pos.z = -pos[2]

        #CoM coordinates in our frame
        self.gps_pos_now.x = self.gps_pos.x  - self.gps_pos_bias[0] 
        self.gps_pos_now.y = self.gps_pos.y  - self.gps_pos_bias[1] 
        self.gps_pos_now.z = self.gps_pos.z  - self.gps_pos_bias[2]

        self.leg_pos_now = np.array([[robot.Leg_lf.position.x, robot.Leg_lf.position.y, robot.Leg_lf.position.z],
                                     [robot.Leg_rf.position.x, robot.Leg_rf.position.y, robot.Leg_rf.position.z],
                                     [robot.Leg_lb.position.x, robot.Leg_lb.position.y, robot.Leg_lb.position.z],
                                     [robot.Leg_rb.position.x, robot.Leg_rb.position.y, robot.Leg_rb.position.z],
                                     ])     #leg position
        
    def forceset(self, force,reaction_force):
        '''
        Quardpueds torques set
        '''
        
        forceclass = Force()

        if not (reaction_force == 1 or -1):
            raise ValueError('reaction_force must be 1 or -1')

        if not np.shape(np.array(force)) == (4,3):
            raise ValueError('force shape must be 4x3')


        force_new = []
        for i in range(4):
            m_force = np.mat([[force[i][0]],[force[i][1]],[force[i][2]]])
            m_R = np.mat(R_Matrix_inv(self.attitude.roll,self.attitude.yaw,self.attitude.pitch))  #RT

            force_new.append(np.squeeze(np.array(m_R * m_force)))

        forceclass.x,forceclass.y,forceclass.z = reaction_force * force_new[0][0], reaction_force * force_new[0][1], reaction_force * force_new[0][2]      
        self.Leg_lf.set_force(forceclass)
        forceclass.x,forceclass.y,forceclass.z = reaction_force * force_new[2][0], reaction_force * force_new[2][1], reaction_force * force_new[2][2]
        self.Leg_lb.set_force(forceclass)
        forceclass.x ,forceclass.y,forceclass.z = reaction_force * force_new[1][0], reaction_force * force_new[1][1], reaction_force * force_new[1][2]
        self.Leg_rf.set_force(forceclass)
        forceclass.x,forceclass.y,forceclass.z = reaction_force * force_new[3][0], reaction_force * force_new[3][1], reaction_force * force_new[3][2]
        self.Leg_rb.set_force(forceclass)


    def forceset_single(self,id,force,reaction_force,*args):
        '''
        Quardpueds single torque set
        '''


        forceclass = Force()
        if not (reaction_force == 1 or -1):
            raise ValueError('reaction_force must be 1 or -1')

        m_force = np.mat([[force[0]],[force[1]],[force[2]]])

        if args[0] =='inv':   #Whether to transform the coordinate
            m_R = np.mat(R_Matrix_inv(self.attitude.roll,self.attitude.yaw,self.attitude.pitch))  
        else :
            m_R = np.mat(R_Matrix(self.attitude.roll,self.attitude.yaw,self.attitude.pitch))  

        force_new  = np.squeeze(np.array(m_R * m_force))
        forceclass.x,forceclass.y,forceclass.z = reaction_force * force_new[0], reaction_force * force_new[1], reaction_force * force_new[2]

        if id ==0:
            self.Leg_lf.set_force(forceclass)
        elif id ==2:
            self.Leg_lb.set_force(forceclass)
        elif id ==1:
            self.Leg_rf.set_force(forceclass)
        elif id ==3:
            self.Leg_rb.set_force(forceclass)



    def posset(self,id,pos):
        '''
        Quardpueds leg position set
        '''


        pos_ = Pos(pos[0], pos[1] , pos[2])

        if id ==0:
            self.Leg_lf.set_position(pos_)
        elif id ==2:
            self.Leg_lb.set_position(pos_)
        elif id ==1:
            self.Leg_rf.set_position(pos_)
        elif id ==3:
            self.Leg_rb.set_position(pos_)


    def stand_up(self): 
        '''
        Quardpueds standing initialization function
        '''


        pos = Pos(0, -self.leg_exp_len , 0)
        self.Leg_lf.set_position(pos)
        self.Leg_lb.set_position(pos)
        self.Leg_rf.set_position(pos)
        self.Leg_rb.set_position(pos)
        #gps bias
        self.gps_pos_bias[0], self.gps_pos_bias[1],self.gps_pos_bias[2] = self.gps_pos.x, self.gps_pos.y -self.body_exp_y, self.gps_pos.z



    def gps_pos_bias_refresh(self):
        '''
        gps bias 
        '''


        self.gps_pos_bias[0], self.gps_pos_bias[1],self.gps_pos_bias[2] = self.gps_pos.x,0, self.gps_pos.z



robot = Dog()
timestep,times = int(robot.getBasicTimeStep()),0

#Force control variable
force = [[0 for j in range(3)] for i in range(4)]


'''
dog_b test part 
'''
from dog_func.record_val import val_rec_balance
from dog_func.stand_balance import balance_keeping

while robot.step(timestep) != -1: 
 
    times += 1
    robot.refresh()

    if(times <  (1000/timestep) * 0.1):
        robot.stand_up()

        pass
    else:
        
        balance_keeping(robot,force_out_2,foot_force_out,force)

        #val_rec_balance(robot,times,timestep,force)
        pass


    