from sdu_controllers.controllers import admittance_controller_position
from kuka_iiwa.iiwa7_urdf import Iiwa

from spatialmath import UnitQuaternion, SO3
import pyFRI as fri
import roboticstoolbox as rtb
import numpy as np
import ctypes as C
import quaternion
import time
import copy

def get_circle_target(pose, timestep, radius=0.055, freq=0.1):
    circ_target = copy.deepcopy(pose)
    circ_target[0] = pose[0] + radius * np.cos((2 * np.pi * freq * timestep))
    circ_target[1] = pose[1] + radius * np.sin((2 * np.pi * freq * timestep))
    print(circ_target)
    return circ_target

class MyClient(fri.LBRClient):
    def __init__(self, adm):   
        super().__init__()
        self.robot = Iiwa()
        self.Tep = self.robot.fkine([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])
        self.done_once = False
        self.counter = 0
        self.adm_controller =adm


    ######## Reading, Encoding and Decoding for FRI ################
 
    def get_joints(self):
        '''Gets Joints poses from FRI and decodes them into np.array '''
        measured_joints = fri.LBRClient.robotState(self).getMeasuredJointPosition()
        p = (C.c_double * 7).from_address(int(measured_joints))
        list(p)
        self.joints = np.array(p)
        return 

    def set_joints(self, goal_joints):
        '''Encodes joints positions from np.array to fri.doubleArray to send to FRI'''
        values = fri.doubleArray(7)
        for i in range(7):
            values[i] = goal_joints[i]
        return values

    def get_ext_ft(self):
        '''Gets Force torque for each joint from FRI and decodes them into np.array '''
        ft_measured = fri.LBRClient.robotState(self).getExternalTorque()
        p = (C.c_double * 7).from_address(int(ft_measured))
        list(p)
        self.joints_ft = np.array(p)
        #tested: print("Force Torque: ", self.joints_ft)
        return
    
    ######## Kinematics and Dynamics  ################
    
    def fw_kinematics(self): 
        self.current_tep = self.robot.fkine(self.joints)
        self.x = self.current_tep.t[0]
        self.y = self.current_tep.t[1]
        self.z = self.current_tep.t[2]
        np.set_printoptions(suppress = True)
        #test: print("Cartesian Orientation: ", cartesian_or)
        quat = UnitQuaternion(self.current_tep)
        #test: print("Cartesian Orientation in Quaternion: ", quat)
        self.quat_w = quat.s
        self.quat_x = quat.v[0]
        self.quat_y = quat.v[1]
        self.quat_z = quat.v[2]
        return
    
    def inv_kinematics(self, goal_pos, quat_goal):
        '''This function does the inverse kinematics of the current goal'''
        uquat_goal = UnitQuaternion(quat_goal[0], [quat_goal[1], quat_goal[2], quat_goal[3]])
        goal_SE3 = uquat_goal.SE3() 
        goal_SE3.t[0] = goal_pos[0]
        goal_SE3.t[1] = goal_pos[1] 
        goal_SE3.t[2] = goal_pos[2] 
        #Create SE3 structure from data pos and data_or
        solution = self.robot.ik_LM(goal_SE3, q0=self.joints)
        goal_joints = solution[0] # get joint poses
        #print("Goal Joints", goal_joints)
        return goal_joints
    
    def ft_at_end_effector_in_world_frame(self):
        self.jacobian = self.robot.jacob0(self.joints)
        pseudo_inv = np.linalg.pinv(self.jacobian.T)
        self.force_torque = np.dot(pseudo_inv, self.joints_ft)
        #tested print("Force_Torque_Base_Frame: ", self.force_torque)
        #? Seems correct
        return
    

    ######## Run the controller  ################

    def step_controller(self):
        # the input position and orientation is given as tip in base
        self.adm_controller.pos_input = [self.x, self.y, self.z]
        self.adm_controller.rot_input = quaternion.from_float_array([self.quat_w, self.quat_x,self.quat_y, self.quat_z])        
        self.adm_controller.ft_input = np.hstack([self.force_torque[0], self.force_torque[1], self.force_torque[2], 0, 0 ,0])
       
        #????????????????????????? This is for testing purposes ?????????????????????????
        so = SO3(self.Tep) # convert from SE3 to S03 
        quat = UnitQuaternion(so) #convert to quaternion
        x_desired =  get_circle_target(self.Tep.t, self.counter)#[cartesian_pos[0], cartesian_pos[1], cartesian_pos[2]] 
        self.adm_controller.set_desired_frame(x_desired, quaternion.from_float_array([quat.s, quat.v[0], quat.v[1], quat.v[2]]))
        #????????????????????????? This is for testing purposes ?????????????????????????
        
        self.adm_controller.step()
        ## output from controller
        output = self.adm_controller.get_output()
        output_position = output[0:3]
        output_quat = output[3:7]
        return (output_position, output_quat)
    

    ######## Protagonist Function  ################

    def command(self): 
        # start timer
        #start_time = time.time()

        # read joints and force
        self.get_joints() 
        self.get_ext_ft()
        
        # run kinematics and dynamics
        self.fw_kinematics()
        self.ft_at_end_effector_in_world_frame()
        
   
        controller_out = self.step_controller()        
        # Inverse Kinematics 
        joint_goal = self.inv_kinematics(controller_out[0], controller_out[1])
        self.tg = rtb.tools.trajectory.jtraj(self.joints, joint_goal, 8)
        joint_to_send = self.tg.s[1]
        values = self.set_joints(joint_to_send)
        #fri.LBRClient.robotCommand(self).setJointPosition(values)            
        self._robotCommand.setJointPosition(values)
            
        # check timer 
        #print("Execution time: %s seconds <<<<<", (time.time()-start_time))
         
        
if __name__ == "__main__":

    DEFAULT_PORTID = 30200
    # KONI IP address
    IP_ADDR = "192.170.10.2"
    

    #Create Client
    #! Need to check this, maybe set the position in sunrise to make sure we always start from there.
    adm_controller = admittance_controller_position.AdmittanceControllerPosition(start_position=np.array([0,0,0]), start_orientation=np.array([1, 0, 0, 0]),                    
                                  start_ft=np.hstack([0,0,0, 0, 0 ,0])) 
    adm_controller.M = np.diag([22.5, 22.5, 22.5])
    adm_controller.D = np.diag([160.0, 160.0, 160.0])
    adm_controller.K = np.diag([54.0, 54.0, 54.0])
    adm_controller.Mo = np.diag([0.25, 0.25, 0.25])
    adm_controller.Do = np.diag([10.0, 10.0, 10.0])
    adm_controller.Ko = np.diag([10.0, 10.0, 10.0])

    client = MyClient(adm_controller)
    app = fri.ClientApplication(client)
    app.connect(DEFAULT_PORTID, IP_ADDR)
    success = True

    frequency = 100.0  # Hz
    dt = 1 / frequency

    while(success):
        start_time = time.time()

        success = app.step()
        client.counter = client.counter + dt

        print("Execution time: %s seconds <<<<<", (time.time()-start_time))

    #client.adm_controller.stop()
    app.disconnect()



