---
sidebar_position: 13
---

# Chapter 13: Sim-to-Real Transfer

## Introduction

The "reality gap" between simulation and real-world deployment is one of robotics' biggest challenges. This chapter covers techniques to bridge this gap and successfully transfer learned behaviors to physical robots.

:::tip Learning Objectives
- Understand the reality gap
- Apply domain randomization
- Perform system identification  
- Use sim-to-real transfer techniques
- Implement online adaptation
- Deploy and validate policies
:::

## The Reality Gap

**Issues causing sim-to-real mismatch:**
- Inaccurate physics models
- Sensor noise not modeled
- Actuator dynamics simplified
- Contact/friction mismatches
- Latency and delays ignored

## Domain Randomization

### Comprehensive Randomization

```python
import numpy as np

class DomainRandomizer:
    def __init__(self):
        # Physical properties
        self.mass_range = (0.7, 1.3)
        self.friction_range = (0.4, 1.6)
        self.damping_range = (0.5, 2.0)
        self.restitution_range = (0.0, 0.5)
        
        # Visual properties
        self.texture_range = (0.5, 1.5)
        self.lighting_range = (500, 5000)
        
        # Sensor noise
        self.imu_noise_std = (0.01, 0.05)
        self.camera_noise_std = (0.0, 0.02)
        
        # Actuator properties
        self.motor_strength_range = (0.8, 1.2)
        self.motor_delay_range = (0.0, 0.02)
    
    def randomize_environment(self, env):
        """Apply comprehensive randomization"""
        
        # Randomize physics
        for i, link in enumerate(env.links):
            # Mass
            mass_scale = np.random.uniform(*self.mass_range)
            link.set_mass(link.base_mass * mass_scale)
            
            # Friction
            friction = np.random.uniform(*self.friction_range)
            link.set_friction(friction)
            
            # Damping
            damping = np.random.uniform(*self.damping_range)
            link.set_damping(damping)
        
        # Randomize visuals
        for light in env.lights:
            intensity = np.random.uniform(*self.lighting_range)
            light.set_intensity(intensity)
        
        # Randomize sensors
        env.imu.noise_std = np.random.uniform(*self.imu_noise_std)
        env.camera.noise_std = np.random.uniform(*self.camera_noise_std)
        
        # Randomize actuators
        for motor in env.motors:
            strength = np.random.uniform(*self.motor_strength_range)
            motor.set_strength_scale(strength)
            
            delay = np.random.uniform(*self.motor_delay_range)
            motor.set_delay(delay)
        
        return env
```

### Curriculum Randomization

```python
class CurriculumRandomizer:
    def __init__(self):
        self.training_steps = 0
        self.difficulty = 0.0
    
    def update_difficulty(self, success_rate):
        """Gradually increase difficulty"""
        if success_rate > 0.8:
            self.difficulty = min(1.0, self.difficulty + 0.1)
        elif success_rate < 0.3:
            self.difficulty = max(0.0, self.difficulty - 0.05)
    
    def get_randomization_ranges(self):
        """Scale randomization with difficulty"""
        return {
            'mass': (1.0 - 0.3*self.difficulty, 1.0 + 0.3*self.difficulty),
            'friction': (1.0 - 0.6*self.difficulty, 1.0 + 0.6*self. difficulty),
            'noise': (0.0, 0.05*self.difficulty)
        }
```

## System Identification

### Parameter Estimation

```python
import scipy.optimize as opt

class SystemIdentifier:
    def __init__(self, robot):
        self.robot = robot
        self.nominal_params = self.get_nominal_parameters()
    
    def collect_data(self, num_trials=100):
        """Collect data from real robot"""
        data = []
        for trial in range(num_trials):
            # Apply random actions
            actions = np.random.randn(self.robot.num_joints) * 0.5
            
            # Record state transitions
            state_before = self.robot.get_state()
            self.robot.apply_actions(actions)
            time.sleep(0.1)
            state_after = self.robot.get_state()
            
            data.append((state_before, actions, state_after))
        
        return data
    
    def estimate_parameters(self, real_data):
        """Optimize sim parameters to match real data"""
        
        def objective(params):
            # Set simulation parameters
            self.set_sim_params(params)
            
            # Simulate same actions
            error = 0
            for state_before, actions, real_state_after in real_data:
                self.robot_sim.set_state(state_before)
                self.robot_sim.apply_actions(actions)
                sim_state_after = self.robot_sim.get_state()
                
                # Calculate prediction error
                error += np.linalg.norm(sim_state_after - real_state_after)
            
            return error / len(real_data)
        
        # Optimize
        result = opt.minimize(
            objective,
            x0=self.nominal_params,
            method='Nelder-Mead'
        )
        
        optimal_params = result.x
        return optimal_params
```

### Learning Forward Dynamics

```python
import torch
import torch.nn as nn

class DynamicsModel(nn.Module):
    """Learn residual dynamics: Δstate = f(state, action)"""
    
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, state_dim)
        )
    
    def forward(self, state, action):
        x = torch.cat([state, action], dim=-1)
        delta_state = self.net(x)
        return state + delta_state

def train_dynamics_model(model, real_data, epochs=100):
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    
    for epoch in range(epochs):
        total_loss = 0
        for state, action, next_state in real_data:
            state_t = torch.FloatTensor(state)
            action_t = torch.FloatTensor(action)
            next_state_t = torch.FloatTensor(next_state)
            
            # Predict
            pred_next_state = model(state_t, action_t)
            
            # Loss
            loss = nn.MSELoss()(pred_next_state, next_state_t)
            
            # Update
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
        
        if epoch % 10 == 0:
            print(f"Epoch {epoch}, Loss: {total_loss:.4f}")
```

## Online Adaptation

### Prior-Based Adaptation

```python
class AdaptivePolicy:
    def __init__(self, base_policy):
        self.base_policy = base_policy
        self.adaptation_module = nn.Linear(16, 64)  # Context encoder
        self.context_buffer = []
    
    def update_context(self, transition):
        """Update context from recent experience"""
        self.context_buffer.append(transition)
        if len(self.context_buffer) > 100:
            self.context_buffer.pop(0)
    
    def get_context_vector(self):
        """Encode recent experience"""
        if len(self.context_buffer) == 0:
            return torch.zeros(16)
        
        # Simple statistics of recent transitions
        states = [t[0] for t in self.context_buffer]
        rewards = [t[2] for t in self.context_buffer]
        
        context = torch.FloatTensor([
            np.mean([s[0] for s in states]),  # Mean of first state dim
            np.std([s[0] for s in states]),   # Std of first state dim
            # ... more statistics
            np.mean(rewards),
            np.std(rewards)
        ])
        
        return context
    
    def act(self, state):
        """Adapt policy based on context"""
        context = self.get_context_vector()
        adapted_features = self.adaptation_module(context)
        
        # Modify base policy with adapted features
        action = self.base_policy(state, adapted_features)
        return action
```

## Deployment Pipeline

### Complete Sim-to-Real Pipeline

```python
class SimToRealPipeline:
    def __init__(self):
        self.stages = [
            'train_in_sim',
            'domain_randomization',
            'system_identification',
            'fine_tuning',
            'real_world_validation',
            'online_adaptation'
        ]
    
    def execute(self):
        # Stage 1: Train in simulation
        print("Training policy in simulation...")
        policy = self.train_in_simulation()
        
        # Stage 2: Domain randomization
        print("Applying domain randomization...")
        randomized_policy = self.apply_randomization(policy)
        
        # Stage 3: System identification
        print("Performing system identification...")
        sim_params = self.system_identification()
        
        # Stage 4: Fine-tuning
        print("Fine-tuning in refined simulation...")
        refined_policy = self.fine_tune(randomized_policy, sim_params)
        
        # Stage 5: Real-world validation
        print("Validating on real robot...")
        success_rate = self.validate_real(refined_policy)
        
        if success_rate < 0.7:
            print("Success rate too low, collecting more data...")
            real_data = self.collect_real_data()
            refined_policy = self.adapt_with_real_data(refined_policy, real_data)
        
        # Stage 6: Deploy with online adaptation
        print("Deploying with online adaptation...")
        adaptive_policy = AdaptivePolicy(refined_policy)
        
        return adaptive_policy
```

## Hardware Deployment

### ROS 2 Deployment Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class PolicyDeploymentNode(Node):
    def __init__(self, policy):
        super().__init__('policy_deployment')
        
        self.policy = policy
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.policy.to(self.device)
        self.policy.eval()
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)
        
        # Control frequency
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        
        self.current_state = None
        
    def state_callback(self, msg):
        """Receive joint states"""
        self.current_state = np.array(msg.position + msg.velocity)
    
    def control_loop(self):
        """Main control loop"""
        if self.current_state is None:
            return
        
        # Get action from policy
        state_tensor = torch.FloatTensor(self.current_state).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action_tensor = self.policy(state_tensor)
        
        action = action_tensor.cpu().numpy().flatten()
        
        # Publish command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = action.tolist()
        self.cmd_pub.publish(cmd_msg)

def main():
    rclpy.init()
    
    # Load trained policy
    policy = torch.load('trained_policy.pt')
    
    # Create and run node
    node = PolicyDeploymentNode(policy)
    rclpy.spin(node)
```

## Safety and Monitoring

### Safety Wrapper

```python
class SafetyWrapper:
    def __init__(self, policy, robot):
        self.policy = policy
        self.robot = robot
        
        # Safety limits
        self.joint_pos_limits = robot.get_joint_limits()
        self.joint_vel_limits = robot.get_velocity_limits()
        self.joint_torque_limits = robot.get_torque_limits()
        
        # Emergency stop
        self.emergency_stop = False
    
    def clip_action(self, action):
        """Clip actions to safe range"""
        return np.clip(action, 
                      -self.joint_torque_limits,
                      self.joint_torque_limits)
    
    def check_safety(self, state):
        """Check if state is safe"""
        positions = state[:self.robot.num_joints]
        velocities = state[self.robot.num_joints:2*self.robot.num_joints]
        
        # Check position limits
        pos_safe = np.all(positions > self.joint_pos_limits[:, 0]) and \
                   np.all(positions < self.joint_pos_limits[:, 1])
        
        # Check velocity limits
        vel_safe = np.all(np.abs(velocities) < self.joint_vel_limits)
        
        return pos_safe and vel_safe
    
    def act(self, state):
        """Get safe action"""
        if not self.check_safety(state):
            print("WARNING: Unsafe state detected!")
            self.emergency_stop = True
            return np.zeros(self.robot.num_joints)
        
        action = self.policy(state)
        safe_action = self.clip_action(action)
        
        return safe_action
```

## Summary

✅ Domain randomization techniques  
✅ System identification  
✅ Online adaptation  
✅ Deployment pipeline  
✅ Safety mechanisms  

## Practice Exercises

1. Implement domain randomization for your robot
2. Collect real-world data and perform system ID
3. Deploy policy with safety wrapper
4. Measure sim-to-real transfer success

## Next Chapter

Module 4 begins! In **Chapter 14**, we'll explore:
- Humanoid kinematics and dynamics
- Forward/inverse kinematics
- Dynamics equations
- Whole-body control

👉 [Continue to Module 4: Chapter 14 →](../module-4/chapter-14.md)

---

:::tip Module 3 Complete!
You now master AI perception, RL, and sim-to-real transfer. Ready for advanced humanoid control!
:::
