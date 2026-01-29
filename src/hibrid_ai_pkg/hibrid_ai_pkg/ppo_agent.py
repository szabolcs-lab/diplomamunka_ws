import numpy as np
import torch
from actor_critic_network import ActorCriticNetwork
from ppo_memory import PPOMemory
import torch.nn.functional as F

class PPOAgent:
    def __init__(self, n_actions=2, input_dims=(15,), gamma=0.99, gae_lambda=0.95, policy_clip=0.2, n_epochs=10):
        
        self.gamma = gamma
        self.policy_clip = policy_clip
        self.n_epochs = n_epochs
        self.gae_lambda = gae_lambda
        
        self.actor_critic_network = ActorCriticNetwork(input_dims=input_dims, n_actions=n_actions)
        self.ppo_memory = PPOMemory()
        
    def store_value(self, state, action, prob, val, reward, done):
        self.ppo_memory.store_data(state, action, prob, val, reward, done)