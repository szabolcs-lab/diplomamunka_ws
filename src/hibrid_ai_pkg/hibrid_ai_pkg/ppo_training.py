import os
import shutil
import numpy as np
import torch
import torch.nn.functional as F

from .actor_critic_network import ActorCriticNetwork
from .ppo_memory import PPOMemory

import time


class PPOTraining:
    """
    PPO (Proximal Policy Optimization) tanítási segédosztály.

    Feladatok:
      lépésenkéénti tapasztalatok eltárolása (state, action, log_prob, reward, done)
      epizód végén (vagy elegendő mintaszámnál) policy frissítése PPO-val
      GAE (Generalized Advantage Estimation) számítása előnyökhöz (advantages) és célértékekhez (returns)
      modellek mentése futtatásonkét elkülönített mappába
    """

    def __init__(self, state_dim: int, action_dim: int = 2):
        # PPO/GAE hiperparaméterek
        self.gamma = 0.99
        self.gae_lambda = 0.95
        self.clip = 0.15 #0.1  # 0.2
        self.k_epochs = 6  # 4 ez volt a kiindulás, 6

        # Loss súlyok
        self.entropy_coef = 0.001 #0.005  # 0.01
        self.value_coef = 0.5
        self.max_grad_norm = 0.5

        # Minimum lépésszám frissítéshez
        self.min_update_steps = 512 #256 #64

        # Mentés / run mappa - ezt törölnöm kell, majd!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.run_id = time.strftime("%Y-%m-%d_%H%M%S")
        self.save_dir = f"./ppo_runs/run_{self.run_id}"
        os.makedirs(self.save_dir, exist_ok=True)

        # Ments gyakorisága
        self.save_gyakorisag = 1

        # epizód számláló (run-on belül)
        self.episode = 0
        
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

        print(f"PPO run_id={self.run_id} save_dir={self.save_dir}")

    def store(self, state, action, log_prob, reward, done):
        """
        Tapasztalat tárolása a  PPO memóriába.

        Paraméterek:
          state: megfigyelés / állapot (pl. lidar + célinfo stb.)
          action: végrehajtott akció (folytonos vektor)
          log_prob: a policy által számolt log-valószínűség az adott action-re
          reward: jutalom az adott lépésre
          done: terminális jelző (epizód vége)
        """
        done_flag = 1.0 if bool(done) else 0.0

        # log_prob - scalar
        if isinstance(log_prob, torch.Tensor):
            log_prob_numpy = log_prob.detach().cpu().numpy()
            log_prob_numpy = np.array(log_prob_numpy).reshape(-1)
            log_prob_scalar = float(np.sum(log_prob_numpy))
        else:
            log_prob_numpy = np.array(log_prob).reshape(-1)
            log_prob_scalar = float(np.sum(log_prob_numpy))

        # action
        if isinstance(action, torch.Tensor):
            action_out = action.detach().cpu().numpy()
        else:
            action_out = np.array(action, dtype=np.float32)

        state_out = np.array(state, dtype=np.float32)

        self.memory.store_data(state_out, action_out, log_prob_scalar, float(reward), done_flag)

    def finish_episode(self):
        """
        Epizód vége.
        """
        # epizód számláló nő MINDEN esetben
        self.episode = self.episode + 1

        # ha kevés adat van, akkor nem updatelek
        if len(self.memory.states) < self.min_update_steps:
            self.memory.clear_data()
            print(f"[PPO] ep={self.episode} kevés adat ({len(self.memory.states)}), nincs update.")
            return

        # tanítás
        self.update_policy()

        # mentés időnkét
        if self.episode % self.save_gyakorisag == 0:
            self.save()

        #memória ürítés
        self.memory.clear_data()

    def update_policy(self):
        """
        PPO policy frissítés a memóriában összegyűjtött rollout adatok alapján.
        """
        states_tensor = torch.tensor(np.array(self.memory.states), dtype=torch.float32, device=self.device)
        actions_tensor = torch.tensor(np.array(self.memory.actions), dtype=torch.float32, device=self.device)
        old_log_probs_tensor = torch.tensor(np.array(self.memory.log_probs), dtype=torch.float32, device=self.device)

        reward_numpy = np.array(self.memory.rewards, dtype=np.float32)
        dones_numpy = np.array(self.memory.is_terminals, dtype=np.float32)

        # old_policy - value becslés (critic)
        with torch.no_grad():
            _, value_tensor = self.old_policy(states_tensor)
            value_numpy = value_tensor.squeeze(-1).cpu().numpy()

        advantages_numpy, returns_numpy = self.gae(reward_numpy, value_numpy, dones_numpy)

        advantages_tensor = torch.tensor(advantages_numpy, dtype=torch.float32, device=self.device)
        returns_tensor = torch.tensor(returns_numpy, dtype=torch.float32, device=self.device)

        # Advantage normalizálás, stabilabb tanulás
        advantages_tensor = (advantages_tensor - advantages_tensor.mean()) / (advantages_tensor.std(unbiased=False) + 1e-8)

        for _ in range(self.k_epochs):
            action_distribution, new_value_tensor = self.policy(states_tensor)
            new_value_tensor = new_value_tensor.squeeze(-1)

            new_log_probs_tensor = action_distribution.log_prob(actions_tensor).sum(-1)
            ratios = torch.exp(new_log_probs_tensor - old_log_probs_tensor)

            surr1 = ratios * advantages_tensor
            surr2 = torch.clamp(ratios, 1.0 - self.clip, 1.0 + self.clip) * advantages_tensor
            actor_loss = -torch.min(surr1, surr2).mean()

            critic_loss = F.mse_loss(new_value_tensor, returns_tensor)

            entropy = action_distribution.entropy().sum(-1).mean()

            loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy

            self.policy.actor_optim.zero_grad()
            self.policy.critic_optim.zero_grad()
            loss.backward()

            torch.nn.utils.clip_grad_norm_(self.policy.actor.parameters(), self.max_grad_norm)
            torch.nn.utils.clip_grad_norm_(self.policy.critic.parameters(), self.max_grad_norm)

            self.policy.actor_optim.step()
            self.policy.critic_optim.step()

        self.copy_policy()

        print(f"PPO epizód={self.episode} steps={len(reward_numpy)} avgR={float(np.mean(reward_numpy)):.3f}")


    def gae(self, rewards: np.ndarray, values: np.ndarray, dones: np.ndarray):
        num_steps = len(rewards)

        advantages = np.zeros(num_steps, dtype=np.float32)
        returns = np.zeros(num_steps, dtype=np.float32)

        A_t = 0.0

        for i in reversed(range(num_steps)):
            next_value = 0.0 if i == num_steps - 1 else values[i + 1]
            non_terminal_mask = 1.0 - dones[i]

            td_error = rewards[i] + self.gamma * next_value * non_terminal_mask - values[i]
            A_t = td_error + self.gamma * self.gae_lambda * non_terminal_mask * A_t

            advantages[i] = A_t
            returns[i] = advantages[i] + values[i]

        return advantages, returns

    def copy_policy(self):
        """
        Az old_policy frissítése az aktuális policy paramétereivel.
        """
        self.old_policy.actor.load_state_dict(self.policy.actor.state_dict())
        self.old_policy.critic.load_state_dict(self.policy.critic.state_dict())

    def save(self):
        """
        Policy checkpoint mentése.
        """
        path = os.path.join(self.save_dir, f"ppo_ep_{self.episode}.pth")

        old_save_file = self.policy.save_file
        self.policy.save_file = path
        self.policy.save_in_file()
        self.policy.save_file = old_save_file

        print(f"PPO elmentve: {path}")

        latest_path = os.path.join(self.save_dir, "latest.pth")
        shutil.copyfile(path, latest_path)

        #run-hoz kötött latest, ha később másolgatnám a mappákat
        latest_run_path = os.path.join(self.save_dir, f"latest_{self.run_id}.pth")
        shutil.copyfile(path, latest_run_path)

        print(f"PPO latest-je frissítve: {latest_path}")