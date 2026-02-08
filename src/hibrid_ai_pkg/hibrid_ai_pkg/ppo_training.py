import os
import numpy as np
import torch
import torch.nn.functional as F

from .actor_critic_network import ActorCriticNetwork
from .ppo_memory import PPOMemory


class PPOTraining:
    def __init__(self, state_dim=15, action_dim=2):
        # alap hiperparaméterek (hardcode jelleg)
        self.gamma = 0.99
        self.gae_lambda = 0.95
        self.clip = 0.2
        self.k_epochs = 4

        self.entropy_coef = 0.01
        self.value_coef = 0.5
        self.max_grad_norm = 0.5

        self.min_update_steps = 128

        self.save_dir = "./ppo_models"
        self.save_freq = 100
        self.episode = 0

        self.device = torch.device("cpu")

        self.policy = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)
        self.old_policy = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)

        self.policy.to(self.device)
        self.old_policy.to(self.device)

        self.memory = PPOMemory()

        os.makedirs(self.save_dir, exist_ok=True)
        self.copy_policy()

    def store(self, state, action, log_prob, reward, done):
        self.memory.store_data(state, action, log_prob, reward, done)

    def finish_episode(self):
        # ha kevés adat van, akkor nem tanulunk belőle
        if len(self.memory.states) < self.min_update_steps:
            self.memory.clear_data()
            self.episode += 1
            return

        self.update()

        self.episode += 1
        if self.episode % self.save_freq == 0:
            self.save()

        self.memory.clear_data()

    def update(self):
        # --- adatok ---
        states = torch.tensor(np.array(self.memory.states), dtype=torch.float32, device=self.device)
        actions = torch.tensor(np.array(self.memory.actions), dtype=torch.float32, device=self.device)
        old_log_probs = torch.tensor(np.array(self.memory.log_probs), dtype=torch.float32, device=self.device)

        rewards = np.array(self.memory.rewards, dtype=np.float32)
        dones = np.array(self.memory.is_terminals, dtype=np.float32)

        # --- critic értékek a régi policyből ---
        with torch.no_grad():
            _, values = self.old_policy(states)
            values = values.squeeze(-1).cpu().numpy()

        # --- advantage + return (GAE) ---
        advantages, returns = self.gae(rewards, values, dones)

        advantages = torch.tensor(advantages, dtype=torch.float32, device=self.device)
        returns = torch.tensor(returns, dtype=torch.float32, device=self.device)

        # sima normalizálás (ne legyen túl nagy)
        advantages = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)

        # --- PPO tanítás ---
        for _ in range(self.k_epochs):
            dist, new_values = self.policy(states)
            new_values = new_values.squeeze(-1)

            new_log_probs = dist.log_prob(actions).sum(-1)
            ratios = torch.exp(new_log_probs - old_log_probs)

            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.clip, 1 + self.clip) * advantages
            actor_loss = -torch.min(surr1, surr2).mean()

            critic_loss = F.mse_loss(new_values, returns)

            entropy = dist.entropy().sum(-1).mean()

            loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy

            self.policy.actor_optim.zero_grad()
            self.policy.critic_optim.zero_grad()
            loss.backward()

            torch.nn.utils.clip_grad_norm_(self.policy.actor.parameters(), self.max_grad_norm)
            torch.nn.utils.clip_grad_norm_(self.policy.critic.parameters(), self.max_grad_norm)

            self.policy.actor_optim.step()
            self.policy.critic_optim.step()

        self.copy_policy()

        print(f"[PPO] episode={self.episode} steps={len(rewards)} avgR={float(np.mean(rewards)):.3f}")

    def gae(self, rewards, values, dones):
        T = len(rewards)
        adv = np.zeros(T, dtype=np.float32)
        ret = np.zeros(T, dtype=np.float32)

        gae = 0.0
        next_value = 0.0

        for t in reversed(range(T)):
            not_done = 1.0 - dones[t]
            delta = rewards[t] + self.gamma * next_value * not_done - values[t]
            gae = delta + self.gamma * self.gae_lambda * not_done * gae

            adv[t] = gae
            ret[t] = adv[t] + values[t]

            next_value = values[t]

        return adv, ret

    def copy_policy(self):
        self.old_policy.actor.load_state_dict(self.policy.actor.state_dict())
        self.old_policy.critic.load_state_dict(self.policy.critic.state_dict())

    def save(self):
        path = os.path.join(self.save_dir, f"ppo_ep_{self.episode}.pth")
        old = self.policy.save_file
        self.policy.save_file = path
        self.policy.save_in_file()
        self.policy.save_file = old
        print(f"[PPO] mentve: {path}")
