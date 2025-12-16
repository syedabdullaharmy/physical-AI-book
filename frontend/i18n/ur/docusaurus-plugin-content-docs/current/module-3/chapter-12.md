---
sidebar_position: 12
---

# Chapter 12: Reinforcement Learning for Robots

## Introduction

Reinforcement Learning (RL) enables robots to learn complex behaviors through trial and error. This chapter covers modern RL algorithms for robot control and how to deploy them effectively.

:::tip Learning Objectives
- Understand RL fundamentals (MDP, rewards, policies)
- Implement PPO and SAC algorithms
- Train robot policies in Isaac Gym
- Use domain randomization
- Deploy trained policies on real robots
- Implement curriculum learning
:::

## RL Fundamentals

### Markov Decision Process (MDP)

```python
class RobotEnvironment:
    """
    MDP for robot control
    - State (s): joint positions, velocities, goal position
    - Action (a): joint torques or velocities
    - Reward (r): distance to goal, energy penalty
    - Next state (s'): resulting state after action
    """
    
    def __init__(self):
        self.state_dim = 34  # 17 joints × 2 (pos + vel)
        self.action_dim = 17  # 17 joint torques
    
    def reset(self):
        """Initialize environment, return initial state"""
        self.robot_state = self.get_random_initial_state()
        return self.robot_state
    
    def step(self, action):
        """
        Execute action, return (next_state, reward, done, info)
        """
        # Apply action to robot
        self.apply_torques(action)
        
        # Step physics simulation
        self.sim_step()
        
        # Get new state
        next_state = self.get_state()
        
        # Calculate reward
        reward = self.compute_reward(next_state, action)
        
        # Check if episode done
        done = self.is_terminal(next_state)
        
        return next_state, reward, done, {}
    
    def compute_reward(self, state, action):
        """Reward function design"""
        # Distance to goal (negative for reward)
        goal_dist = np.linalg.norm(state[:3] - self.goal_pos)
        distance_reward = -goal_dist
        
        # Energy penalty (encourage efficiency)
        energy_penalty = -0.01 * np.sum(np.square(action))
        
        # Alive bonus
        alive_bonus = 1.0
        
        # Total reward
        reward = distance_reward + energy_penalty + alive_bonus
        
        return reward
```

## PPO (Proximal Policy Optimization)

PPO is the most popular RL algorithm for robotics:

### PPO Implementation

```python
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super().__init__()
        
        # Shared feature extractor
        self.features = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )
        
        # Policy head (actor)
        self.actor_mean = nn.Linear(hidden_dim, action_dim)
        self.actor_log_std = nn.Parameter(torch.zeros(action_dim))
        
        # Value head (critic)
        self.critic = nn.Linear(hidden_dim, 1)
    
    def forward(self, state):
        features = self.features(state)
        return features
    
    def act(self, state):
        """Sample action from policy"""
        features = self.forward(state)
        
        # Policy distribution
        mean = self.actor_mean(features)
        std = torch.exp(self.actor_log_std)
        dist = Normal(mean, std)
        
        # Sample action
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(-1)
        
        # Value estimate
        value = self.critic(features)
        
        return action, log_prob, value
    
    def evaluate(self, state, action):
        """Evaluate actions"""
        features = self.forward(state)
        
        mean = self.actor_mean(features)
        std = torch.exp(self.actor_log_std)
        dist = Normal(mean, std)
        
        log_prob = dist.log_prob(action).sum(-1)
        entropy = dist.entropy().sum(-1)
        value = self.critic(features)
        
        return log_prob, entropy, value

class PPO:
    def __init__(self, state_dim, action_dim):
        self.policy = ActorCritic(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=3e-4)
        
        self.clip_epsilon = 0.2
        self.value_coef = 0.5
        self.entropy_coef = 0.01
        self.gamma = 0.99
        self.gae_lambda = 0.95
    
    def update(self, rollouts):
        """PPO update step"""
        states, actions, old_log_probs, returns, advantages = rollouts
        
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        for _ in range(10):  # K epochs
            # Evaluate actions with current policy
            log_probs, entropy, values = self.policy.evaluate(states, actions)
            
            # Policy loss (clipped objective)
            ratio = torch.exp(log_probs - old_log_probs)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - self.clip_epsilon,
                               1 + self.clip_epsilon) * advantages
            policy_loss = -torch.min(surr1, surr2).mean()
            
            # Value loss
            value_loss = 0.5 * (returns - values).pow(2).mean()
            
            # Entropy bonus (exploration)
            entropy_loss = -entropy.mean()
            
            # Total loss
            loss = policy_loss + self.value_coef * value_loss + \
                   self.entropy_coef * entropy_loss
            
            # Update
            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.policy.parameters(), 0.5)
            self.optimizer.step()
```

### Training Loop

```python
def train_ppo(env, ppo_agent, num_iterations=1000):
    """Train PPO agent"""
    for iteration in range(num_iterations):
        # Collect rollouts
        states_list = []
        actions_list = []
        log_probs_list = []
        rewards_list = []
        values_list = []
        dones_list = []
        
        state = env.reset()
        episode_reward = 0
        
        for step in range(2048):  # Rollout length
            state_tensor = torch.FloatTensor(state)
            
            with torch.no_grad():
                action, log_prob, value = ppo_agent.policy.act(state_tensor)
            
            next_state, reward, done, _ = env.step(action.numpy())
            
            states_list.append(state)
            actions_list.append(action)
            log_probs_list.append(log_prob)
            rewards_list.append(reward)
            values_list.append(value)
            dones_list.append(done)
            
            episode_reward += reward
            state = next_state
            
            if done:
                state = env.reset()
        
        # Compute returns and advantages (GAE)
        returns, advantages = compute_gae(
            rewards_list, values_list, dones_list,
            ppo_agent.gamma, ppo_agent.gae_lambda
        )
        
        # Convert to tensors
        rollouts = (
            torch.FloatTensor(states_list),
            torch.stack(actions_list),
            torch.stack(log_probs_list),
            returns,
            advantages
        )
        
        # Update policy
        ppo_agent.update(rollouts)
        
        if iteration % 10 == 0:
            print(f"Iteration {iteration}, Reward: {episode_reward:.2f}")
```

## SAC (Soft Actor-Critic)

SAC is an off-policy algorithm with automatic temperature tuning:

```python
class SACAgent:
    def __init__(self, state_dim, action_dim):
        # Actor network
        self.actor = GaussianPolicy(state_dim, action_dim)
        
        # Two Q-networks (for double Q-learning)
        self.critic1 = QNetwork(state_dim, action_dim)
        self.critic2 = QNetwork(state_dim, action_dim)
        
        # Target networks
        self.critic1_target = QNetwork(state_dim, action_dim)
        self.critic2_target = QNetwork(state_dim, action_dim)
        
        # Copy weights
        self.critic1_target.load_state_dict(self.critic1.state_dict())
        self.critic2_target.load_state_dict(self.critic2.state_dict())
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=3e-4)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=3e-4)
        
        # Temperature parameter
        self.log_alpha = torch.tensor(np.log(0.2), requires_grad=True)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=3e-4)
        
        self.gamma = 0.99
        self.tau = 0.005
        self.target_entropy = -action_dim
    
    def update(self, batch):
        states, actions, rewards, next_states, dones = batch
        
        # Update Q-functions
        with torch.no_grad():
            next_actions, next_log_probs = self.actor.sample(next_states)
            target_q1 = self.critic1_target(next_states, next_actions)
            target_q2 = self.critic2_target(next_states, next_actions)
            target_q = torch.min(target_q1, target_q2)
            target_q = rewards + self.gamma * (1 - dones) * \
                       (target_q - self.log_alpha.exp() * next_log_probs)
        
        current_q1 = self.critic1(states, actions)
        current_q2 = self.critic2(states, actions)
        
        critic1_loss = F.mse_loss(current_q1, target_q)
        critic2_loss = F.mse_loss(current_q2, target_q)
        
        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()
        
        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()
        
        # Update actor
        new_actions, log_probs = self.actor.sample(states)
        q1 = self.critic1(states, new_actions)
        q2 = self.critic2(states, new_actions)
        q = torch.min(q1, q2)
        
        actor_loss = (self.log_alpha.exp() * log_probs - q).mean()
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        
        # Update temperature
        alpha_loss = -(self.log_alpha.exp() * 
                      (log_probs + self.target_entropy).detach()).mean()
        
        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()
        
        # Soft update target networks
        self.soft_update(self.critic1_target, self.critic1)
        self.soft_update(self.critic2_target, self.critic2)
    
    def soft_update(self, target, source):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(
                target_param.data * (1.0 - self.tau) + param.data * self.tau
            )
```

## Isaac Gym Integration

### Creating Isaac Gym Environment

```python
from isaacgym import gymapi, gymutil
import torch

class HumanoidEnv:
    def __init__(self, num_envs=4096):
        # Create gym
        self.gym = gymapi.acquire_gym()
        
        # Create sim
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / 60.0
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
        
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
        
        # Create envs
        self.num_envs = num_envs
        self.envs = []
        self.humanoid_handles = []
        
        spacing = 2.0
        env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)
        
        asset_file = "humanoid.urdf"
        asset_options = gymapi.AssetOptions()
        asset_options.angular_damping = 0.01
        asset_options.max_angular_velocity = 100.0
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_EFFORT
        
        humanoid_asset = self.gym.load_asset(
            self.sim, "", asset_file, asset_options
        )
        
        for i in range(num_envs):
            env = self.gym.create_env(self.sim, env_lower, env_upper, 8)
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0.0, 0.0, 1.0)
            
            humanoid_handle = self.gym.create_actor(
                env, humanoid_asset, pose, "humanoid", i, 1
            )
            
            self.envs.append(env)
            self.humanoid_handles.append(humanoid_handle)
    
    def reset(self):
        """Reset all environments"""
        for i, env in enumerate(self.envs):
            # Reset pose
            self.gym.set_actor_root_state_tensor_indexed(
                self.sim,
                gymtorch.unwrap_tensor(self.initial_root_states),
                gymtorch.unwrap_tensor(torch.tensor([i], dtype=torch.int32)),
                len([i])
            )
        
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        
        return self.get_observations()
    
    def step(self, actions):
        """Step all environments"""
        # Apply actions
        self.gym.set_dof_actuation_force_tensor(
            self.sim,
            gymtorch.unwrap_tensor(actions)
        )
        
        # Simulate
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        
        # Get observations and rewards
        obs = self.get_observations()
        rewards = self.compute_rewards()
        dones = self.compute_dones()
        
        return obs, rewards, dones, {}
```

### Vectorized Training

```python
def train_isaac_gym(env, ppo_agent, num_iterations=10000):
    """Train with massive parallelization"""
    for iteration in range(num_iterations):
        obs = env.reset()  # (num_envs, obs_dim)
        
        for step in range(128):
            # Get actions for all envs at once
            with torch.no_grad():
                actions, log_probs, values = ppo_agent.policy.act(obs)
            
           # Step all envs simultaneously
            next_obs, rewards, dones, _ = env.step(actions)
            
            # Store transitions
            # ... (same as before but vectorized)
            
            obs = next_obs
        
        # Update policy (same as PPO)
        ppo_agent.update(rollouts)
        
        if iteration % 100 == 0:
            avg_reward = rewards.mean().item()
            print(f"Iteration {iteration}, Avg Reward: {avg_reward:.2f}")
```

## Domain Randomization

```python
class RandomizedEnv:
    def __init__(self, base_env):
        self.env = base_env
        
        # Randomization ranges
        self.mass_range = (0.8, 1.2)
        self.friction_range = (0.5, 1.5)
        self.damping_range = (0.5, 2.0)
    
    def randomize_physics(self):
        """Randomize physics properties"""
        for i in range(self.env.num_envs):
            # Randomize masses
            for link in range(self.env.num_links):
                mass_scale = np.random.uniform(*self.mass_range)
                self.env.set_link_mass(i, link, 
                                      self.env.default_masses[link] * mass_scale)
            
            # Randomize friction
            friction = np.random.uniform(*self.friction_range)
            self.env.set_friction(i, friction)
            
            # Randomize damping
            damping = np.random.uniform(*self.damping_range)
            self.env.set_damping(i, damping)
    
    def reset(self):
        self.randomize_physics()
        return self.env.reset()
```

## Summary

✅ PPO and SAC algorithms  
✅ Isaac Gym for massive parallelization  
✅ Domain randomization  
✅ Reward shaping  
✅ Vectorized training  

## Practice Exercises

1. Train PPO agent for humanoid walking
2. Implement custom reward function
3. Add domain randomization
4. Compare PPO vs SAC performance

## Next Chapter

In **Chapter 13**, we'll explore:
- Sim-to-real transfer techniques
- System identification
- Reality gap mitigation
- Deployment strategies

👉 [Continue to Chapter 13 →](./chapter-13.md)
