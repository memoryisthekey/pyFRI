import sys
import os
import psutil
import argparse
import pyFRI as fri
from pyFRI.tools.state_estimators import (
    JointStateEstimator,
    FRIExternalTorqueEstimator,
    WrenchEstimatorTaskOffset,
    TaskSpaceStateEstimator,
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
import copy
from robot import load_kuka_iiwa7


if fri.FRI_VERSION_MAJOR == 1:
    POSITION = fri.EClientCommandMode.POSITION
elif fri.FRI_VERSION_MAJOR == 2:
    POSITION = fri.EClientCommandMode.JOINT_POSITION

def get_circle_target(pose, timestep, radius=0.1, freq=0.05):
    circ_target = copy.deepcopy(pose)
    circ_target[0] = pose[0] + radius * np.cos((2 * np.pi * freq * timestep))
    circ_target[1] = pose[1] + radius * np.sin((2 * np.pi * freq * timestep))
    return circ_target

class HandGuideClient(fri.LBRClient):
    def __init__(self, lbr_ver):
        super().__init__()
        self.robot = Iiwa()
        self.counter = 0
        self.ee_link = "lbr_link_ee"
        self.robot_optas = load_kuka_iiwa7([1])
        ################################################################
        self.joint_state_estimator = JointStateEstimator(self)
        self.external_torque_estimator = FRIExternalTorqueEstimator(self)
        self.wrench_estimator = WrenchEstimatorTaskOffset(
            self,
            self.joint_state_estimator,
            self.external_torque_estimator,
            self.robot_optas,
            self.ee_link,
        )
        self.wrench_filter = ExponentialStateFilter()
        self.task_space_estimator = TaskSpaceStateEstimator(self, self.joint_state_estimator, self.robot_optas, self.ee_link)


        self.Tep = self.robot.fkine([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])
        x_desired = get_circle_target(self.Tep.t, self.counter)
        self.adm_controller = admittance_controller_position.AdmittanceControllerPosition(start_position=x_desired, start_orientation=np.array([1, 0, 0, 0]),                    
                                  start_ft=np.hstack([0,0,0, 0, 0 ,0])) 
        self.adm_controller.M = np.diag([22.5, 22.5, 22.5])
        self.adm_controller.D = np.diag([70.0, 70.0, 70.0]) #0.5*np.diag([160.0, 160.0, 160.0])
        self.adm_controller.K = np.diag([10.0, 10.0, 10.0])
        self.adm_controller.Mo = np.diag([0.25, 0.25, 0.25])
        self.adm_controller.Do = np.diag([10.0, 10.0, 10.0])
        self.adm_controller.Ko = np.diag([0.0, 0.0, 0.0])

    ######## Reading, Encoding and Decoding for FRI ################
 
    def get_joints(self):
        '''Gets Joints poses from FRI and decodes them into np.array '''
        measured_joints = self.robotState().getMeasuredJointPosition()
        self.joints = np.array(measured_joints)
        return 

    def set_joints(self, goal_joints):
        '''Encodes joints positions from np.array to fri.doubleArray to send to FRI'''
        #values = fri.doubleArray(7)
        
        #for i in range(7):
        values = goal_joints
        return values
    
    ######## Kinematics and Dynamics  ################
    
    def fw_kinematics(self): 
        self.position = self.task_space_estimator.get_transform()
        print("Position: ", self.position)
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
    
    def get_ft(self):
        if not self.wrench_estimator.ready():
            self.wrench_estimator.update()
        wr = self.wrench_estimator.get_wrench()
        self.wf = self.wrench_filter.filter(wr)
        #print("Force_Torque: ", self.wf)
        return

    ######## Run the controller  ################

    def step_controller(self):
        # the input position and orientation is given as tip in base
        #self.adm_controller.pos_input = [self.x, self.y, self.z]
        ##print("Controller Input", self.adm_controller.pos_input)
        #self.adm_controller.rot_input = quaternion.from_float_array([self.quat_w, self.quat_x,self.quat_y, self.quat_z])        
        #self.adm_controller.ft_input = np.hstack([self.force_torque[0], self.force_torque[1], self.force_torque[2], 0, 0 ,0])
       #
        ##????????????????????????? This is for testing purposes ?????????????????????????
        #so = SO3(self.Tep) # convert from SE3 to S03 
        #quat = UnitQuaternion(so) #convert to quaternion
        #x_desired =  get_circle_target(self.Tep.t, self.counter)#[cartesian_pos[0], cartesian_pos[1], cartesian_pos[2]] 
        #self.adm_controller.set_desired_frame(x_desired, quaternion.from_float_array([quat.s, quat.v[0], quat.v[1], quat.v[2]]))
        ##????????????????????????? This is for testing purposes ?????????????????????????
#
        ##print("x_desired", x_desired)
        ## Step controller
        #self.adm_controller.step()
#
        ### output from controller
        #output = self.adm_controller.get_output()
        #output_position = output[0:3]
        #output_quat = output[3:7]
        return #(output_position, output_quat)

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
        
        # run kinematics and dynamics
        self.fw_kinematics()
        self.get_ft()
 
        #controller_out = self.step_controller()    
        self.counter = self.counter + 0.01
    
        # Inverse Kinematics 
        #joint_goal = self.inv_kinematics(controller_out[0], controller_out[1])

        #self.robotCommand().setJointPosition(values) 
        self.q = self.robotState().getIpoJointPosition()
        self.command_position()


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

    # Set application real-time priority
    os_used = sys.platform
    process = psutil.Process(os.getpid())

    if os_used == "win32":  # Windows (either 32-bit or 64-bit)
        process.nice(psutil.REALTIME_PRIORITY_CLASS)
    elif os_used == "linux":  # linux
        rt_app_priority = 80
    param = os.sched_param(rt_app_priority)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
    except OSError:
        print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
    else:
        print("Process real-time priority set to: %u" % rt_app_priority)

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
