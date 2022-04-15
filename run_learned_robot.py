import numpy as np
import time
from common.IMU import IMU
from common.Controller import Controller
from common.JoystickInterface import JoystickInterface
from common.State import State
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

# import rsl_rl module
from policy.rsl_rl.modules import ActorCritic
from policy.rsl_rl.algorithms import PPO

import torch
import torch.nn as nn
import torch.functional as F


def get_policy(path, obs_dim = 235, act_dim=12, actor_hidden_dims=[512, 256, 128], critic_hidden_dims=[512, 256, 128]):
    """
    Re-use policy learned from ISAAC Gym.

    Parameters:
    -----------
    path: relative path to the .pt weight file (str)
    obs_dim: size of the observations vector (int)
    act_dim: size of the actions vector (int)
    actor_hidden_dims: size of each layer of the actor neural network (list)
    critic_hidden_dims: size of each layer of the critic neural network (list)
    """
    obs_dim_actor = obs_dim
    obs_dim_critic = obs_dim

    if not torch.cuda.is_available():
        device = torch.device('cpu')
    else:
        device = torch.device('cuda')

    loaded_dict = torch.load(path, map_location=device)

    actor_critic = ActorCritic(obs_dim_actor, obs_dim_critic, act_dim, actor_hidden_dims=actor_hidden_dims, critic_hidden_dims=critic_hidden_dims).to(device)
    alg = PPO(actor_critic, device=device)

    actor_critic.load_state_dict(loaded_dict["model_state_dict"])
    current_learning_iteration = loaded_dict["iter"]
    actor_critic.eval()  # switch to evaluation mode (dropout for example)
    actor_critic.to(device)
    return actor_critic.act_inference#self.alg.actor_critic.act_inference

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
            command = joystick_interface.get_command(state)
            joystick_interface.set_color(config.ps4_deactivated_color)
            if command.activate_event == 1:
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
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                break

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            #controller.run(state, command)
            state = TODO
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

    main()
