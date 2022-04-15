import numpy as np
import time
import glob
import os

# import rsl_rl module
from policy.rsl_rl.modules import ActorCritic
from policy.rsl_rl.algorithms import PPO

import torch
import torch.nn as nn
import torch.functional as F


def get_policy(path, device, obs_dim=235, act_dim=12, actor_hidden_dims=[512, 256, 128], critic_hidden_dims=[512, 256, 128]):
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

    loaded_dict = torch.load(path, map_location=device)

    actor_critic = ActorCritic(obs_dim_actor, obs_dim_critic, act_dim, actor_hidden_dims=actor_hidden_dims, critic_hidden_dims=critic_hidden_dims).to(device)
    alg = PPO(actor_critic, device=device)

    actor_critic.load_state_dict(loaded_dict["model_state_dict"])
    current_learning_iteration = loaded_dict["iter"]
    actor_critic.eval()  # switch to evaluation mode (dropout for example)
    actor_critic.to(device)
    return actor_critic.act_inference#self.alg.actor_critic.act_inference

def main(policy_path=None):
    """Main program
    """
    
    if not torch.cuda.is_available():
        device = torch.device('cpu')
    else:
        device = torch.device('cuda')

    # Load Model
    print("Loading policy ...")
    model = get_policy(policy_path, device)
    print("Done ...")

    time_list = []
    for i in range(1000):
        obs = torch.randn(235).to(device)
        start_time = time.time()
        out = model(obs)
        time_list.append(time.time() - start_time)
    
    print("average inference time: " + str(sum(time_list) / 1000))

if __name__ == "__main__":

    policy_path = "./policy/pupper/*"
    models = [file for file in glob.glob(policy_path) if "model" in file]
    last_models_path = models[-1]
    main(policy_path=last_models_path)
