import pybullet as p
import pybullet_data
import time
import numpy as np

from sim.IMU import IMU
from sim.encoders import Encoders
from sim.Sim import Sim
from common.Controller import Controller
from common.Command import Command
from common.JoystickInterface import JoystickInterface
from common.State import State
from sim.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

# import rsl_rl module
from policy.rsl_rl.modules import ActorCritic
from policy.rsl_rl.algorithms import PPO

import torch
import torch.nn as nn
import torch.functional as F

import os
import glob

def get_policy(path, obs_dim = 235, act_dim=12, actor_hidden_dims=[512, 256, 128], critic_hidden_dims=[512, 256, 128]):

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

  
def main(default_velocity=np.zeros(2), default_yaw_rate=0.0):
  
    # Create config
    config = Configuration()
    config.z_clearance = 0.02
    sim = Sim(xml_path="sim/pupper_pybullet_out.xml")
    hardware_interface = HardwareInterface(sim.model, sim.joint_indices)

    # Load Model
    model = get_policy(policy_path)

    # Create imu handle
    imu = IMU()
    
    # Create simulated encoders
    encoders = Encoders()


    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics,)
    state = State()
    command = Command()

    # Emulate the joystick inputs required to activate the robot
    command.activate_event = 1
    controller.run(state, command)
    command.activate_event = 0
    command.trot_event = 1
    controller.run(state, command)
    command = Command()  # zero it out

    # Apply a constant command. # TODO Add support for user input or an external commander
    command.horizontal_velocity = default_velocity
    command.yaw_rate = default_yaw_rate

    # The joystick service is linux-only, so commenting out for mac
    # print("Creating joystick listener...")
    # joystick_interface = JoystickInterface(config)
    # print("Done.")

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # Run the simulation
    timesteps = 240 * 60 * 10  # simulate for a max of 10 minutes

    # Sim seconds per sim step
    sim_steps_per_sim_second = 240
    sim_dt = 1.0 / sim_steps_per_sim_second
    last_control_update = 0
    start = time.time()

    for steps in range(timesteps):
        sim_time_elapsed = sim_dt * steps
        if sim_time_elapsed - last_control_update > config.dt:
            last_control_update = sim_time_elapsed

            # Get IMU measurement
            state.quat_orientation = imu.read_orientation()
            
            # Get joint positions and velocities
            joint_pos, joint_vel = encoders.read_pos_vel()
            lin_vel, ang_vel =  imu.read_lin_ang_vel()

            # Step the controller forward by dt
            command  = model(torch.randn(235)).view(3,4)
            controller.send_action(state, command)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(state.joint_angles)

        # Simulate physics for 1/240 seconds (the default timestep)
        sim.step()

        # Performance testing
        elapsed = time.time() - start
        if (steps % 60) == 0:
            print(
                "Sim seconds elapsed: {}, Real seconds elapsed: {}".format(
                    round(sim_time_elapsed, 3), round(elapsed, 3)
                )
            )
            # print("Average steps per second: {0}, elapsed: {1}, i:{2}".format(steps / elapsed, elapsed, i))


if __name__ == "__main__":

    policy_path = "./policy/pupper/*"
    models = [file for file in glob.glob(policy_path) if "model" in file]
    last_models_path = models[-1]

    main(default_velocity=np.array([0, 0]), policy_path=last_models_path)
