import os
import numpy as np
import torch
import torch.nn.functional as F

from actor_critic_network import ActorCriticNetwork
from ppo_memory import PPOMemory


class PPOTraining:
    def __init__(self,state_dim=15, action_dim=2, gamma=0.99, gae_lambda=0.95, clip=0.2, K_epochs=4,entropy_coef=0.01, value_coef=0.5, max_grad_norm=0.5, 
                 save_freq=100, save_dir="./ppo_models",):
        
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip = clip
        self.K_epochs = K_epochs
        self.entropy_coef = entropy_coef
        self.value_coef = value_coef
        self.max_grad_norm = max_grad_norm
        self.save_freq = save_freq
        self.save_dir = save_dir
        self.episode = 0

        #CPU csak
        self.device = torch.device("cpu")

        #hálók
        self.actor_critic = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)
        self.old_actor_critic = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)

        self.actor_critic.to(self.device)
        self.old_actor_critic.to(self.device)

        self.sync_old()

        self.memory = PPOMemory()
        
        os.makedirs(self.save_dir, exist_ok=True)

    def store_transition(self, state, action, log_prob, reward, done):
        self.memory.store_data(state, action, log_prob, reward, done)

    def end_episode(self):
        if len(self.memory.states) > 0:
            self.update()

        self.episode =self.episode + 1
        if self.episode % self.save_freq == 0:
            self.save_checkpoint()

    def update(self):
        #tensorizálás
        states = torch.tensor(np.array(self.memory.states), dtype=torch.float32)
        actions = torch.tensor(np.array(self.memory.actions), dtype=torch.float32)       # raw_action!
        old_log_probs = torch.tensor(np.array(self.memory.log_probs), dtype=torch.float32)

        rewards = np.array(self.memory.rewards, dtype=np.float32)
        dones = np.array(self.memory.is_terminals, dtype=np.float32)

        #critic értékek
        with torch.no_grad():
            _, values = self.old_actor_critic(states)
            values = values.squeeze(-1).numpy()

        #GAE advantage + return
        advantages, returns = self.compute_gae(rewards, values, dones)
        advantages = torch.tensor(advantages, dtype=torch.float32)
        returns = torch.tensor(returns, dtype=torch.float32)

        #advantage normalizálás
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        last_loss = None

        #PPO update
        for _ in range(self.K_epochs):
            distribution, new_values = self.actor_critic(states)
            new_values = new_values.squeeze(-1)

            new_log_probs = distribution.log_prob(actions).sum(-1)
            entropy = distribution.entropy().sum(-1).mean()

            ratios = torch.exp(new_log_probs - old_log_probs)

            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.clip, 1 + self.clip) * advantages
            actor_loss = -torch.min(surr1, surr2).mean()

            critic_loss = F.mse_loss(new_values, returns)

            loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy
            last_loss = loss

            self.actor_critic.actor_optim.zero_grad()
            self.actor_critic.critic_optim.zero_grad()
            loss.backward()

            torch.nn.utils.clip_grad_norm_(self.actor_critic.actor.parameters(), self.max_grad_norm)
            torch.nn.utils.clip_grad_norm_(self.actor_critic.critic.parameters(), self.max_grad_norm)

            self.actor_critic.actor_optim.step()
            self.actor_critic.critic_optim.step()

        self.sync_old()

        avg_r = float(np.mean(rewards))
        print(f"episode={self.episode} | steps={len(rewards)} | avgR={avg_r:.3f} | loss={float(last_loss):.3f}")

        self.memory.clear_data()

    def compute_gae(self, rewards, values, dones):
        num_time_steps = len(rewards)
        advantages = np.zeros(num_time_steps, dtype=np.float32)
        returns = np.zeros(num_time_steps, dtype=np.float32)

        gae = 0.0
        next_value = 0.0

        for nts in reversed(range(num_time_steps)):
            not_done_yet = 1.0 - dones[nts]
            delta = rewards[nts] + self.gamma * next_value * not_done_yet - values[nts]
            gae = delta + self.gamma * self.gae_lambda * not_done_yet * gae

            advantages[nts] = gae
            returns[nts] = advantages[nts] + values[nts]

            next_value = values[nts]

        return advantages, returns

    # lemásoljuk az aktuális policy-t egy régebbibe
    def sync_old(self):
        self.old_actor_critic.actor.load_state_dict(self.actor_critic.actor.state_dict())
        self.old_actor_critic.critic.load_state_dict(self.actor_critic.critic.state_dict())

    def save_checkpoint(self):
        path = os.path.join(self.save_dir, f"ppo_ep_{self.episode}.pth")
        old = self.actor_critic.save_file
        self.actor_critic.save_file = path
        self.actor_critic.save_in_file()
        self.actor_critic.save_file = old
        print(f"PPO checkpoint mentve: {path}")
