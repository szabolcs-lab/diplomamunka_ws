import torch
import torch.nn as nn
from torch.distributions import Normal

class ActorCriticNetwork(nn.Module):
    def __init__(self, n_inputs=15, n_actions=2, save_file='ppo_allapotok'): #n_inputs még változhat!!!!!!
        super().__init__()
        self.save_file = save_file

        #Actor hálózat
        self.actor = nn.Sequential(
            nn.Linear(n_inputs, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, n_actions),
            nn.Tanh()
        )

        #Critic hálózat
        self.critic = nn.Sequential(
            nn.Linear(n_inputs, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

        #Optimalizálás
        self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=3e-4) #0.0003
        self.critic_optim = torch.optim.Adam(self.critic.parameters(), lr=3e-4)

    def forward(self, state):
        value = self.critic(state)
        action_mean = self.actor(state)
        sigma = torch.ones_like(action_mean) * 0.1  # fix szórás
        distribution = Normal(action_mean, sigma)
        return distribution, value

    def save_in_file(self):
        print("Indul a hálózat állapotainak mentése...")
        
        torch.save({'actor': self.actor.state_dict(), 'critic': self.critic.state_dict(), 'actor_optimizer': self.actor_optim.state_dict(), 
                    'critic_optimizer': self.critic_optim.state_dict()}, self.save_file)
        
        print(f"A hálózat állapotai mentve a {self.save_file}-ba!")

    def load_from_file(self):
        print("Indul a halózat mentett állapotainak betöltése...")
        
        allapotok = torch.load(self.save_file)
        
        self.actor.load_state_dict(allapotok['actor'])
        self.critic.load_state_dict(allapotok['critic'])
        self.actor_optim.load_state_dict(allapotok['actor_optimizer'])
        self.critic_optim.load_state_dict(allapotok['critic_optimizer'])
        
        print(f"A hálózat állapotai betöltve a {self.save_file}-ból!")
