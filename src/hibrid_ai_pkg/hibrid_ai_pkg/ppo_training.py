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
    PPO tréner osztály.

    Használat logikája (tipikusan a ROS2 node-odban):
      - Minden timestepnél:
          action, log_prob, value = policy(state)
          környezet léptet -> reward, done
          trainer.store(state, action, log_prob, reward, done)
      - Epizód végén:
          trainer.finish_episode()

    A "környezet" nálad a PathShaper node lesz:
      - state összeállítás
      - action -> shaping paraméter leképezés (offset_m, smooth_strength)
      - reward számítás
      - done feltételek (goal/collision/timeout)
    """

    def __init__(self, state_dim: int, action_dim: int = 2):
        # PPO / GAE hiperparaméterek
        self.gamma = 0.99            # diszkont faktor
        self.gae_lambda = 0.95       # GAE lambda (bias-variance tradeoff)
        self.clip = 0.2              # PPO clip tartomány (pl. 0.2 -> [0.8, 1.2])
        self.k_epochs = 4            # hányszor futunk végig ugyanazon rollout adaton

        # Loss súlyok
        self.entropy_coef = 0.01     # exploráció támogatása (túl kicsi -> hamar beáll, túl nagy -> random)
        self.value_coef = 0.5        # critic loss súlya
        self.max_grad_norm = 0.5     # grad clip (stabilitás)

        # Minimum lépésszám frissítéshez
        # Path shapingnél az epizód lehet rövid, ezért 64 sokszor jobb induláskor, mint 128.
        self.min_update_steps = 64

        # Mentés
        #self.save_dir = "./ppo_models"
        run_id = time.strftime("%Y-%m-%d_%H%M%S")
        self.save_dir = f"./ppo_runs/run_{run_id}"
        self.save_freq = 1         # ennyi epizódonként ment
        self.episode = 0

        # Egyszerűség kedvéért CPU
        self.device = torch.device("cpu")

        # policy: amit tanítunk
        # old_policy: a PPO arányokhoz "referencia" policy (policy frissítés előtti állapot)
        self.policy = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)
        self.old_policy = ActorCriticNetwork(n_inputs=state_dim, n_actions=action_dim)

        self.policy.to(self.device)
        self.old_policy.to(self.device)

        # Rollout memória
        self.memory = PPOMemory()

        os.makedirs(self.save_dir, exist_ok=True)

        # Induláskor old_policy = policy
        self.copy_policy()

    # STORE: 1 timestep adatainak eltárolása
    def store(self, state, action, log_prob, reward, done):
        """
        Ezt hívd minden timestep után.

        state: 1D array/list (state_dim)
        action: 1D array/list (action_dim) vagy torch tensor
        log_prob: skalár (ajánlott), de ha véletlenül vektor -> összegzi
        reward: float
        done: bool vagy 0/1
        """

        # done biztosan 0/1 float legyen
        done_f = 1.0 if bool(done) else 0.0

        # log_prob legyen skalár (összegzett)
        # (pl. ha valahol elrontva 2 dim log_prob-ot tárolok, ez itt "megmenti")
        if isinstance(log_prob, torch.Tensor):
            lp = log_prob.detach().cpu().numpy()
            lp = np.array(lp).reshape(-1)
            log_prob_scalar = float(np.sum(lp))
        else:
            lp = np.array(log_prob).reshape(-1)
            log_prob_scalar = float(np.sum(lp))

        # action egységesítése (list/np/tensor -> sima python list/np)
        if isinstance(action, torch.Tensor):
            action_out = action.detach().cpu().numpy()
        else:
            action_out = np.array(action, dtype=np.float32)

        # state egységesítés
        state_out = np.array(state, dtype=np.float32)

        self.memory.store_data(state_out, action_out, log_prob_scalar, float(reward), done_f)

    # EPIZÓD VÉGE
    def finish_episode(self):
        """
        Ezt hívd epizód végén (goal/collision/timeout).

        Ha túl kevés adat gyűlt (min_update_steps), akkor:
          - nem update-el (különben zajos, instabil lenne)
          - törli a memóriát
        """
        if len(self.memory.states) < self.min_update_steps:
            self.memory.clear_data()
            self.episode += 1
            return

        # Tanítás
        self.update()

        self.episode += 1

        # Mentés időnként
        if self.episode % self.save_freq == 0:
            self.save()

        # Memória ürítése (következő epizód tiszta lappal)
        self.memory.clear_data()

    # PPO UPDATE (a "tanítás" lényege)
    def update(self):
        """
        1) Memory -> torch tensor adatok
        2) Old policyből value(s) becslés
        3) GAE advantage + return számítás
        4) Többszörös epoch PPO frissítés:
           - actor_loss (clipped surrogate)
           - critic_loss (MSE)
           - entropy bonus
        5) Old policy frissítése az új policy-re
        """

        # 1) Memory -> Torch
        states = torch.tensor(np.array(self.memory.states), dtype=torch.float32, device=self.device)
        actions = torch.tensor(np.array(self.memory.actions), dtype=torch.float32, device=self.device)
        old_log_probs = torch.tensor(np.array(self.memory.log_probs), dtype=torch.float32, device=self.device)

        rewards = np.array(self.memory.rewards, dtype=np.float32)
        dones = np.array(self.memory.is_terminals, dtype=np.float32)  # 0/1

        # 2) Value becslés old_policy-ből

        # PPO-nál azért használunk old_policy-t, mert a "ratios" ehhez képest értelmezett
        with torch.no_grad():
            _, values = self.old_policy(states)     # values shape: (N,1)
            values = values.squeeze(-1).cpu().numpy()

        # 3) Advantage + Return (GAE)
        advantages, returns = self.gae(rewards, values, dones)

        advantages = torch.tensor(advantages, dtype=torch.float32, device=self.device)
        returns = torch.tensor(returns, dtype=torch.float32, device=self.device)

        # Advantage normalizálás: stabilabb tanulás (ne legyen óriási skálakülönbség)
        advantages = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)

        # 4) PPO epochs
        for _ in range(self.k_epochs):
            # új policy eloszlás és új value
            dist, new_values = self.policy(states)
            new_values = new_values.squeeze(-1)

            # log_prob az akciókra (összegzett, mert 2 dim akció)
            new_log_probs = dist.log_prob(actions).sum(-1)

            # ratio = pi_new(a|s) / pi_old(a|s)
            ratios = torch.exp(new_log_probs - old_log_probs)

            # PPO clipped objective
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1.0 - self.clip, 1.0 + self.clip) * advantages
            actor_loss = -torch.min(surr1, surr2).mean()

            # Critic loss: V(s) közelítse a return-t
            critic_loss = F.mse_loss(new_values, returns)

            # Entropy: exploráció fenntartására (ne "merevedjen be" túl korán)
            entropy = dist.entropy().sum(-1).mean()

            # Össz loss
            loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy

            # Gradient step
            self.policy.actor_optim.zero_grad()
            self.policy.critic_optim.zero_grad()
            loss.backward()

            # grad clip a stabilitásért
            torch.nn.utils.clip_grad_norm_(self.policy.actor.parameters(), self.max_grad_norm)
            torch.nn.utils.clip_grad_norm_(self.policy.critic.parameters(), self.max_grad_norm)

            self.policy.actor_optim.step()
            self.policy.critic_optim.step()

        # 5) Old policy frissítése
        self.copy_policy()

        print(f"[PPO] episode={self.episode} steps={len(rewards)} avgR={float(np.mean(rewards)):.3f}")

    # GAE (Generalized Advantage Estimation)
    def gae(self, rewards: np.ndarray, values: np.ndarray, dones: np.ndarray):
        """
        rewards[t] : azonnali jutalom
        values[t]  : V(s_t) becslés (critic)
        dones[t]   : 0/1 jelzi, hogy t-nél véget ért-e az epizód

        Kimenet:
          adv[t] : advantage
          ret[t] : return = adv[t] + values[t]
        """
        T = len(rewards)
        adv = np.zeros(T, dtype=np.float32)
        ret = np.zeros(T, dtype=np.float32)

        gae_val = 0.0
        next_value = 0.0  # epizód végén 0

        # visszafelé számolunk
        for t in reversed(range(T)):
            not_done = 1.0 - dones[t]  # ha done=1 - not_done=0

            # TD error
            delta = rewards[t] + self.gamma * next_value * not_done - values[t]

            # GAE rekurzió
            gae_val = delta + self.gamma * self.gae_lambda * not_done * gae_val

            adv[t] = gae_val
            ret[t] = adv[t] + values[t]

            next_value = values[t]

        return adv, ret

    # Old policy = policy másolása
    def copy_policy(self):
        """
        PPO-hoz kell: old_policy a policy frissítés előtti állapotot tartsa.
        """
        self.old_policy.actor.load_state_dict(self.policy.actor.state_dict())
        self.old_policy.critic.load_state_dict(self.policy.critic.state_dict())


    # Mentés
    def save(self):
        """
        Ment:
          - policy háló (actor+critic+optimizer állapotok)
          - "latest.pth" mindig a legutolsó mentésre mutat
        """
        path = os.path.join(self.save_dir, f"ppo_ep_{self.episode}.pth")

        # save_file mezőt használja
        old = self.policy.save_file
        self.policy.save_file = path
        self.policy.save_in_file()
        self.policy.save_file = old

        print(f"[PPO] mentve: {path}")

        latest_path = os.path.join(self.save_dir, "latest.pth")
        shutil.copyfile(path, latest_path)
        print(f"[PPO] latest frissítve: {latest_path}")
