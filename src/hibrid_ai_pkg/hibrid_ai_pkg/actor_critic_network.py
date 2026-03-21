import torch
import torch.nn as nn
from torch.distributions import Normal


class ActorCriticNetwork(nn.Module):
    def __init__(self,n_inputs, n_actions= 2, save_file="ppo.pt",sigma = 0.02 ): #0.05, 0.1, 0.03
        super().__init__()
        self.save_file = save_file
        self.sigma = sigma

        # Actor háló
        self.actor = nn.Sequential(nn.Linear(n_inputs, 256), nn.ReLU(),
                                   nn.Linear(256, 128),nn.ReLU(), 
                                   nn.Linear(128, 64),nn.ReLU(),
                                   nn.Linear(64, n_actions),nn.Tanh())#[-1,1]

        # Critic háló
        self.critic = nn.Sequential(nn.Linear(n_inputs, 256), nn.ReLU(),
                                    nn.Linear(256, 128),nn.ReLU(),
                                    nn.Linear(128, 64),nn.ReLU(),
                                    nn.Linear(64, 1))

        # Optimizer
        self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic_optim = torch.optim.Adam(self.critic.parameters(), lr=1e-3) #3e-4 ezzel kezdtem


    #Visszaad action_distribution, state_value...
    def forward(self, state: torch.Tensor):    
        if state.dim() == 1:
            state = state.unsqueeze(0)  #(1, n_inputs)

        state_value = self.critic(state)    #(1,1)
        action_mean = self.actor(state)     #(1, n_actions)

        action_std_tensor  = torch.ones_like(action_mean) * self.sigma
        action_distribution  = Normal(action_mean, action_std_tensor )

        return action_distribution , state_value


    #Teljes állapot mentése
    def save_in_file(self):
        print("Hálózat mentése indul...")
        torch.save({"actor": self.actor.state_dict(),"critic": self.critic.state_dict(),"actor_optimizer": self.actor_optim.state_dict(),
                    "critic_optimizer": self.critic_optim.state_dict(), "sigma": self.sigma}, self.save_file)
        
        print(f"Mentve: {self.save_file}")


    #Teljes állaopt betöltése
    def load_from_file(self):
        print("Hálózat betöltése indul...")
        data = torch.load(self.save_file, map_location="cpu")

        self.actor.load_state_dict(data["actor"])
        self.critic.load_state_dict(data["critic"])
        self.actor_optim.load_state_dict(data["actor_optimizer"])
        self.critic_optim.load_state_dict(data["critic_optimizer"])

        if "sigma" in data:
            self.sigma = float(data["sigma"])

        print(f"Betöltve: {self.save_file}")
