import os
import shutil
import numpy as np
import torch
import torch.nn.functional as F

from .actor_critic_network import ActorCriticNetwork
from .ppo_memory import PPOMemory

import time


class PPOTraining:
    def __init__(self, state_dim: int, action_dim: int = 2):
        # PPO / GAE hiperparaméterek
        self.gamma = 0.99
        self.gae_lambda = 0.95
        self.clip = 0.2
        self.k_epochs = 4 #4 ez volt a kiindulás, 6 

        # Loss súlyok
        self.entropy_coef = 0.01
        self.value_coef = 0.5
        self.max_grad_norm = 0.5

        # Minimum lépésszám frissítéshez
        self.min_update_steps = 64

        #Mentés / run mappa - ezt törölnöm kell, majd!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.run_id = time.strftime("%Y-%m-%d_%H%M%S")
        self.save_dir = f"./ppo_runs/run_{self.run_id}"
        os.makedirs(self.save_dir, exist_ok=True)

        # Mentsünk pl. 5 epizódonként (de lehet 1 is)
        self.save_freq = 1

        # epizód számláló (run-on belül)
        self.episode = 0

        # Egyszerűség kedvéért CPU
        self.device = torch.device("cpu")

        # policy + old_policy
        self.policy = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)
        self.old_policy = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)

        self.policy.to(self.device)
        self.old_policy.to(self.device)

        # memória
        self.memory = PPOMemory()

        # induláskor old_policy = policy
        self.copy_policy()

        print(f"[PPO] run_id={self.run_id} save_dir={self.save_dir}")

    def store(self, state, action, log_prob, reward, done):
        done_f = 1.0 if bool(done) else 0.0

        # log_prob - scalar
        if isinstance(log_prob, torch.Tensor):
            lp = log_prob.detach().cpu().numpy()
            lp = np.array(lp).reshape(-1)
            log_prob_scalar = float(np.sum(lp))
        else:
            lp = np.array(log_prob).reshape(-1)
            log_prob_scalar = float(np.sum(lp))

        # action
        if isinstance(action, torch.Tensor):
            action_out = action.detach().cpu().numpy()
        else:
            action_out = np.array(action, dtype=np.float32)

        state_out = np.array(state, dtype=np.float32)

        self.memory.store_data(state_out, action_out, log_prob_scalar, float(reward), done_f)

    def finish_episode(self):
        """
        Epizód vége. Ha van elég adat - update + mentés.
        """
        # epizód számláló nő MINDEN esetben (így a fájlnevek sosem ismétlődnek run-on belül)
        self.episode += 1

        # ha kevés adat van, akkor nincs update
        if len(self.memory.states) < self.min_update_steps:
            self.memory.clear_data()
            print(f"[PPO] ep={self.episode} kevés adat ({len(self.memory.states)}), nincs update.")
            return

        # tanítás
        self.update()

        # mentés időnként
        if self.episode % self.save_freq == 0:
            self.save()

        # memória ürítés
        self.memory.clear_data()

    def update(self):
        states = torch.tensor(np.array(self.memory.states), dtype=torch.float32, device=self.device)
        actions = torch.tensor(np.array(self.memory.actions), dtype=torch.float32, device=self.device)
        old_log_probs = torch.tensor(np.array(self.memory.log_probs), dtype=torch.float32, device=self.device)

        rewards = np.array(self.memory.rewards, dtype=np.float32)
        dones = np.array(self.memory.is_terminals, dtype=np.float32)

        with torch.no_grad():
            _, values = self.old_policy(states)
            values = values.squeeze(-1).cpu().numpy()

        advantages, returns = self.gae(rewards, values, dones)

        advantages = torch.tensor(advantages, dtype=torch.float32, device=self.device)
        returns = torch.tensor(returns, dtype=torch.float32, device=self.device)

        advantages = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)

        for _ in range(self.k_epochs):
            dist, new_values = self.policy(states)
            new_values = new_values.squeeze(-1)

            new_log_probs = dist.log_prob(actions).sum(-1)
            ratios = torch.exp(new_log_probs - old_log_probs)

            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1.0 - self.clip, 1.0 + self.clip) * advantages
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

        print(f"[PPO] ep={self.episode} steps={len(rewards)} avgR={float(np.mean(rewards)):.3f}")

    def gae(self, rewards: np.ndarray, values: np.ndarray, dones: np.ndarray):
        T = len(rewards)
        adv = np.zeros(T, dtype=np.float32)
        ret = np.zeros(T, dtype=np.float32)

        gae_val = 0.0
        next_value = 0.0

        for t in reversed(range(T)):
            not_done = 1.0 - dones[t]
            delta = rewards[t] + self.gamma * next_value * not_done - values[t]
            gae_val = delta + self.gamma * self.gae_lambda * not_done * gae_val

            adv[t] = gae_val
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

        latest_path = os.path.join(self.save_dir, "latest.pth")
        shutil.copyfile(path, latest_path)

        # extra: run-hoz kötött latest, ha később másolgatnám a mappákat
        latest_run_path = os.path.join(self.save_dir, f"latest_{self.run_id}.pth")
        shutil.copyfile(path, latest_run_path)

        print(f"[PPO] latest frissítve: {latest_path}")
