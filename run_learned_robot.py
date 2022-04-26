import numpy as np
import time
from common.IMU import IMU
from common.Controller import Controller
from common.JoystickInterface import JoystickInterface
from common.State import State
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

from policy.utils import get_policy

import torch
import torch.nn as nn
import torch.functional as F

import glob
import os

TARGET = [0.1, 0.0]
ACTION_SCALE = 0.25
LIN_VEL_SCALE = 2.0
ANG_VEL_SCALE = 0.25
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
CLIP_OBS = 100.0
CLIP_ACT = 100.0
DEFAULT_JOINT_POS = torch.Tensor([-0.15, 0.5, -1.0, 0.15, 0.5, -1.0,
                                  -0.15, 0.7, -1.0, 0.15, 0.7, -1.0])

def main(use_imu=False, policy_path=None):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()

    # Load Model
    print("Loading policy ...")
    model = get_policy(policy_path)
    print("Done ...")

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # Wait until the activate button has been pressed
    while True:
        print("Waiting for L1 to activate robot.")
        while True:
            joystick_command = joystick_interface.get_command(state)
            joystick_interface.set_color(config.ps4_deactivated_color)
            if joystick_command.activate_event == 1:
                break
            time.sleep(0.1)
        print("Robot activated.")
        joystick_interface.set_color(config.ps4_color)

        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            joystick_command = joystick_interface.get_command(state)
            if joystick_command.activate_event == 1:
                print("Deactivating Robot")
                break

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            #controller.run(state, command)
            horizontal_velocity = joystick_command.horizontal_velocity
            yaw_rate = joystick_command.yaw_rate

            obs = TODO
            command  = model(obs).view(3,4)
            # WATCH OUT HERE IS STATE
            controller.send_action(state, command)
            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(state.joint_angles)

if __name__ == "__main__":

    policy_path = "./policy/pupper/*"
    models = [file for file in glob.glob(policy_path) if "model" in file]
    last_models_path = models[-1]
    main(policy_path=last_models_path, obs_dim=48)
