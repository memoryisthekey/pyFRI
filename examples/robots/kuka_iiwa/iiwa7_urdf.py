#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.Robot import Robot
import os
import getpass

user = getpass.getuser()
class Iiwa(Robot):
    """
    Class that imports a LBR URDF model

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):
        cwd = os.getcwd()  
        if(cwd == '/home/'+user+'/fri-python-wrapper/kuka_iiwa'):
            links, name, urdf_string, urdf_filepath = self.URDF_read(
                cwd + "/urdf/iiwa7.urdf.xacro"
            )
        else:
            links, name, urdf_string, urdf_filepath = self.URDF_read(
                cwd + "/kuka_iiwa/urdf/iiwa7.urdf.xacro"
            )

        #links, name, urdf_string, urdf_filepath = self.URDF_read(
        #    "/home/sdu/docker/shared/fri-python-wrapper/kuka_iiwa/urdf/iiwa7.urdf.xacro"
        #)
 

        super().__init__(
            links,
            name=name,
            manufacturer="Kuka",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            # gripper_links=elinks[9]
        )

        # self.qdlim = np.array([
        #     2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0])

        #self.qr = np.array([0, -0.3, 0, -1.9, 0, 1.5, np.pi / 4])
        #self.qz = np.zeros(7)

        #self.addconfiguration("qr", self.qr)
        #self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = Iiwa()
    print(robot)
    print("------------------------------------")
    print(robot.ets())