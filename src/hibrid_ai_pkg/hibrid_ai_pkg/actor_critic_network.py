import torch
import torch.nn as nn
from torch.distributions import Normal


class ActorCriticNetwork(nn.Module):
    def __init__(self,n_inputs: int, n_actions: int = 2, save_file: str = "ppo.pt",sigma: float = 0.05 ): #0.1
        super().__init__()
        self.save_file = save_file
        self.sigma = sigma

        # Actor háló
        self.actor = nn.Sequential(nn.Linear(n_inputs, 256), nn.ReLU(),
                                   nn.Linear(256, 128),nn.ReLU(), 
                                   nn.Linear(128, 64),nn.ReLU(),
                                   nn.Linear(64, n_actions),nn.Tanh())# action E [-1,1]

        # Critic háló
        self.critic = nn.Sequential(nn.Linear(n_inputs, 256), nn.ReLU(),
                                    nn.Linear(256, 128),nn.ReLU(),
                                    nn.Linear(128, 64),nn.ReLU(),
                                    nn.Linear(64, 1))

        # Optimizer)
        self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic_optim = torch.optim.Adam(self.critic.parameters(), lr=3e-4)

    def forward(self, state: torch.Tensor):    
        if state.dim() == 1:
            state = state.unsqueeze(0)  # (1, n_inputs)

        state_value = self.critic(state)          # (1,1)
        action_mean = self.actor(state)     # (1, n_actions)

        action_std_tensor  = torch.ones_like(action_mean) * self.sigma
        action_distribution  = Normal(action_mean, action_std_tensor )

        return action_distribution , state_value

    def save_in_file(self):
        print("Hálózat mentése indul...")
        torch.save({"actor": self.actor.state_dict(),"critic": self.critic.state_dict(),"actor_optimizer": self.actor_optim.state_dict(),
                    "critic_optimizer": self.critic_optim.state_dict(), "sigma": self.sigma}, self.save_file)
        
        print(f"Mentve: {self.save_file}")

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


def action_to_shaping(action_tensor, max_offset_meter: float = 0.20):
    if action_tensor.dim() == 2:
        action_tensor = action_tensor.squeeze(0)

    offset_action  = float(torch.clamp(action_tensor[0], -1.0, 1.0).item())
    smooth_action  = float(torch.clamp(action_tensor[1], -1.0, 1.0).item())

    path_offset_meter  = offset_action  * max_offset_meter
    smoothing_strength  = (smooth_action  + 1.0) * 0.5

    return path_offset_meter , smoothing_strength
