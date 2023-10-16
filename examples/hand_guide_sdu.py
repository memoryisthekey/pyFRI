import sys
import math
import argparse
import pyFRI as fri
from pyFRI.tools.state_estimators import (
    JointStateEstimator,
    FRIExternalTorqueEstimator,
    WrenchEstimatorTaskOffset,
)
from pyFRI.tools.filters import ExponentialStateFilter
import ctypes as C

from sdu_controllers.controllers import admittance_controller_position
from kuka_iiwa.iiwa7_urdf import Iiwa

from spatialmath import UnitQuaternion, SO3
import roboticstoolbox as rtb
import numpy as np
import quaternion
import time
import logging 
from datetime import datetime

if fri.FRI_VERSION_MAJOR == 1:
    POSITION = fri.EClientCommandMode.POSITION
elif fri.FRI_VERSION_MAJOR == 2:
    POSITION = fri.EClientCommandMode.JOINT_POSITION


class HandGuideClient(fri.LBRClient):
    def __init__(self, lbr_ver):
        super().__init__()
        self.robot = Iiwa()

        self.Tep = self.robot.fkine([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])
        self.adm_controller = admittance_controller_position.AdmittanceControllerPosition(start_position=np.array([0,0,0]), start_orientation=np.array([1, 0, 0, 0]),                    
                                  start_ft=np.hstack([0,0,0, 0, 0 ,0])) 
        self.adm_controller.M = np.diag([22.5, 22.5, 22.5])
        self.adm_controller.D = np.diag([70.0, 70.0, 70.0]) #0.5*np.diag([160.0, 160.0, 160.0])
        self.adm_controller.K = np.diag([0.0, 0.0, 0.0])
        self.adm_controller.Mo = np.diag([0.25, 0.25, 0.25])
        self.adm_controller.Do = np.diag([10.0, 10.0, 10.0])
        self.adm_controller.Ko = np.diag([0.0, 0.0, 0.0])
        #logging.basicConfig(filename='logs/force_logfile_'+datetime.now().strftime("%d_%m_%Y-%Hh%Mm%Ss")+'.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        #self.logger = logging.getLogger() 
    ######## Reading, Encoding and Decoding for FRI ################
 
    def get_joints(self):
        '''Gets Joints poses from FRI and decodes them into np.array '''
        measured_joints = self.robotState().getMeasuredJointPosition()
        #p = (C.c_double * 7).from_address(int(measured_joints))
        #list(p)
        print(measured_joints)
        self.joints = np.array(measured_joints)
        return 

    def set_joints(self, goal_joints):
        '''Encodes joints positions from np.array to fri.doubleArray to send to FRI'''
        #values = fri.doubleArray(7)
        
        #for i in range(7):
        values = goal_joints
        return values

    def get_ext_ft(self):
        '''Gets Force torque for each joint from FRI and decodes them into np.array '''
        ft_measured = self.robotState().getExternalTorque()
        #p = (C.c_double * 7).from_address(int(ft_measured))
        #list(p)
        self.joints_ft = np.array(ft_measured)
        #tested: print("Force Torque: ", self.joints_ft)
        return
    
    ######## Kinematics and Dynamics  ################
    
    def fw_kinematics(self): 
        self.current_tep = self.robot.fkine(self.joints)
        #? Could Optimize
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
        #self.logger.info("Position:{0}".format(self.current_tep.t))
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
        #print("Force_Torque_Base_Frame: ", self.force_torque)
        #self.logger.info("Force Torque: {0}".format(self.force_torque))
        #print("Fy: ",self.force_torque[1])
        #print("Fz: ",self.force_torque[2])
        return

    ######## Run the controller  ################

    def step_controller(self):
        # the input position and orientation is given as tip in base
        self.adm_controller.pos_input = [self.x, self.y, self.z]
        #print("Controller Input", self.adm_controller.pos_input)
        self.adm_controller.rot_input = quaternion.from_float_array([self.quat_w, self.quat_x,self.quat_y, self.quat_z])        
        self.adm_controller.ft_input = np.hstack([self.force_torque[0], self.force_torque[1], self.force_torque[2], 0, 0 ,0])
       
        #????????????????????????? This is for testing purposes ?????????????????????????
        so = SO3(self.Tep) # convert from SE3 to S03 
        quat = UnitQuaternion(so) #convert to quaternion
        x_desired =  self.Tep.t #[cartesian_pos[0], cartesian_pos[1], cartesian_pos[2]] 
        self.adm_controller.set_desired_frame(x_desired, quaternion.from_float_array([quat.s, quat.v[0], quat.v[1], quat.v[2]]))
        #????????????????????????? This is for testing purposes ?????????????????????????

        # Step controller
        self.adm_controller.step()

        ## output from controller
        output = self.adm_controller.get_output()
        output_position = output[0:3]
        output_quat = output[3:7]
        return (output_position, output_quat)

    def command_position(self):
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

    def waitForCommand(self):
        if self.robotState().getClientCommandMode() != POSITION:
            print(
                f"[ERROR] hand guide example requires {POSITION.name} client command mode"
            )
            raise SystemExit

        self.q = self.robotState().getIpoJointPosition()
        self.command_position()

    def command(self):
        # read joints and force
        self.get_joints() 
        self.get_ext_ft()
        
        # run kinematics and dynamics
        self.fw_kinematics()
        self.ft_at_end_effector_in_world_frame()
 
        controller_out = self.step_controller()        
        # Inverse Kinematics 
        joint_goal = self.inv_kinematics(controller_out[0], controller_out[1])
        #self.tg = rtb.tools.trajectory.jtraj(self.joints, joint_goal, 5)
        #self.tg1 = np.linspace(self.joints, joint_goal, 5)
        #print("TG1: ", self.tg1[0])
        #joint_to_send = self.tg1[1]
        #joint_to_send = self.tg.s[1]
        values = self.set_joints(joint_goal)
        #print(joint_to_send)
        self.robotCommand().setJointPosition(values) 
        #self.command_position()


def get_arguments():
    parser = argparse.ArgumentParser(description="LRBJointSineOverlay example.")
    parser.add_argument(
        "--hostname",
        dest="hostname",
        default="192.170.10.2",
        help="The hostname used to communicate with the KUKA Sunrise Controller.",
    )
    parser.add_argument(
        "--port",
        dest="port",
        type=int,
        default=30200,
        help="The port number used to communicate with the KUKA Sunrise Controller.",
    )
    parser.add_argument(
        "--lbr-ver",
        dest="lbr_ver",
        type=int,
        default=7,
        #choices=[7, 14],
        #required=True,
        help="The KUKA LBR Med version number.",
    )

    return parser.parse_args()


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    args = get_arguments()
    client = HandGuideClient(args.lbr_ver)
    app = fri.ClientApplication(client)
    success = app.connect(args.port, args.hostname)

    if not success:
        print("Connection to KUKA Sunrise controller failed.")
        return 1

    try:
        while success:
            start_time = time.time()

            success = app.step()

            print("Execution time: %s seconds <<<<<", (time.time()-start_time))


            if client.robotState().getSessionState() == fri.ESessionState.IDLE:
                break

    except KeyboardInterrupt:
        pass

    except SystemExit:
        pass

    finally:
        app.disconnect()
        print("Goodbye")

    return 0


if __name__ == "__main__":
    sys.exit(main())
