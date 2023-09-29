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


from admittance import AdmittanceController

import numpy as np
#from paho.mqtt import client as mqtt_client


if fri.FRI_VERSION_MAJOR == 1:
    POSITION = fri.EClientCommandMode.POSITION
elif fri.FRI_VERSION_MAJOR == 2:
    POSITION = fri.EClientCommandMode.JOINT_POSITION


class HandGuideClient(fri.LBRClient):
    def __init__(self, lbr_ver):
        super().__init__()
        self.controller = AdmittanceController(lbr_ver)
        self.joint_state_estimator = JointStateEstimator(self)
        self.external_torque_estimator = FRIExternalTorqueEstimator(self)
        self.wrench_estimator = WrenchEstimatorTaskOffset(
            self,
            self.joint_state_estimator,
            self.external_torque_estimator,
            self.controller.robot,
            self.controller.ee_link,
        )
        self.wrench_filter = ExponentialStateFilter()

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

        self.wrench_estimator.update()
        self.q = self.robotState().getIpoJointPosition()
        self.command_position()

    def command(self):
        if not self.wrench_estimator.ready():
            self.wrench_estimator.update()
            self.robotCommand().setJointPosition(self.q.astype(np.float32))
            return

        # Get robot state
        wr = self.wrench_estimator.get_wrench()
        dt = self.robotState().getSampleTime()

        # Filter wrench
        wf = self.wrench_filter.filter(wr)
        #print("Wrench: ", wf)

        # Compute goal using admittance controller
        self.q = self.controller(self.q, wf, dt)

        # Command robot
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

    args = get_arguments()
    client = HandGuideClient(args.lbr_ver)
    app = fri.ClientApplication(client)
    success = app.connect(args.port, args.hostname)

    if not success:
        print("Connection to KUKA Sunrise controller failed.")
        return 1

    try:
        while success:
            success = app.step()

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
