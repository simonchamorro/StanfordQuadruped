import pybullet as p
import pybullet_data
import time
import numpy as np


from sim.Sim import Sim, PupperSim

# from common.JoystickInterface import JoystickInterface

import torch
import torch.nn as nn
import torch.functional as F

import os
import glob


def main(default_velocity=np.zeros(2), default_yaw_rate=0.0, policy_path=None, use_policy=True):
    # Create config
    sim = PupperSim(xml_path="sim/pupper_pybullet_out.xml", policy_path=policy_path)
    sim.run(verbose=True)


if __name__ == "__main__":

    policy_path = "./policy/pupper/*"
    models = [file for file in glob.glob(policy_path) if "model" in file]
    last_models_path = models[-1]
    main(policy_path=last_models_path, use_policy=True)
