---
sidebar_position: 15
---

# Chapter 15: Bipedal Locomotion and Balance

## Introduction

Bipedal walking is one of robotics' greatest challenges. This chapter covers stability criteria, gait generation, and balance control for humanoid robots.

:::tip Learning Objectives
- Understand ZMP and balance
- Generate stable walking gaits
- Implement preview control
- Use Model Predictive Control (MPC)
- Handle disturbances and push recovery
:::

## Zero Moment Point (ZMP)

### ZMP Theory

The ZMP is the point on the ground where net moment is zero. For stability:
- ZMP must stay inside support polygon
- ZMP = Center of Pressure (CoP) for flat foot contact

```python
import numpy as np

class ZMPCalculator:
    def __init__(self, robot):
        self.robot = robot
        self.g = 9.81
    
    def compute_zmp(self, com_pos, com_acc, foot_forces):
        """
        Compute ZMP position
        ZMP_x = (CoM_x * m*g - sum(f_z * x_i)) / sum(f_z)
        """
        com_x, com_y, com_z = com_pos
        com_ddx, com_ddy, _ = com_acc
        
        total_mass = self.robot.get_total_mass()
        
        # Horizontal forces from acceleration
        F_x = total_mass * com_ddx
        F_y = total_mass * com_ddy
        
        # Vertical force
        F_z = total_mass * self.g
        
        # Moment from com acceleration
        M_x = total_mass * (com_ddy * com_z + com_y * self.g)
        M_y = total_mass * (com_ddx * com_z + com_x * self.g)
        
        # ZMP position
        zmp_x = (com_x * F_z - M_y) / F_z
        zmp_y = (com_y * F_z + M_x) / F_z
        
        return np.array([zmp_x, zmp_y, 0])
    
    def is_stable(self, zmp_pos, support_polygon):
        """Check if ZMP is inside support polygon"""
        from shapely.geometry import Point, Polygon
        
        zmp_point = Point(zmp_pos[0], zmp_pos[1])
        polygon = Polygon(support_polygon)
        
        return polygon.contains(zmp_point)
```

## Gait Generation

### Walking Pattern Generator

```python
class WalkingPatternGenerator:
    def __init__(self):
        self.step_length = 0.15  # meters
        self.step_height = 0.05
        self.step_duration = 0.8  # seconds
        self.double_support_ratio = 0.2
    
    def generate_footsteps(self, num_steps):
        """Generate footstep sequence"""
        footsteps = []
        
        for i in range(num_steps):
            if i % 2 == 0:  # Left foot
                x = i/2 * self.step_length
                y = 0.1  # Foot separation
                foot = 'left'
            else:  # Right foot
                x = (i+1)/2 * self.step_length
                y = -0.1
                foot = 'right'
            
            footsteps.append({
                'foot': foot,
                'position': np.array([x, y, 0]),
                'time': i * self.step_duration
            })
        
        return footsteps
    
    def generate_com_trajectory(self, footsteps, dt=0.01):
        """
        Generate CoM trajectory using preview control
        """
        T = len(footsteps) * self.step_duration
        time_steps = np.arange(0, T, dt)
        
        com_trajectory = []
        
        for t in time_steps:
            # Determine current support foot
            step_idx = int(t / self.step_duration)
            
            if step_idx >= len(footsteps):
                break
            
            current_step = footsteps[step_idx]
            
            # During single support, CoM moves toward next footstep
            if step_idx < len(footsteps) - 1:
                next_step = footsteps[step_idx + 1]
                
                # Linear CoM motion in x
                progress = (t - current_step['time']) / self.step_duration
                com_x = current_step['position'][0] + \
                        progress * (next_step['position'][0] - current_step['position'][0])
                
                # Sinusoidal CoM motion in y for lateral stability
                com_y = 0.05 * np.sin(2 * np.pi * progress)
            else:
                com_x = current_step['position'][0]
                com_y = 0
            
            com_z = 0.8  # Hip height
            
            com_trajectory.append(np.array([com_x, com_y, com_z]))
        
        return np.array(com_trajectory)
    
    def generate_swing_foot_trajectory(self, start_pos, end_pos, duration, current_time):
        """Generate smooth swing foot trajectory"""
        t = current_time / duration
        t = np.clip(t, 0, 1)
        
        # Cubic spline for smooth motion
        s = 3*t**2 - 2*t**3
        
        # Horizontal motion
        pos_xy = start_pos[:2] + s * (end_pos[:2] - start_pos[:2])
        
        # Vertical motion (parabolic)
        pos_z = 4 * self.step_height * t * (1 - t)
        
        return np.array([pos_xy[0], pos_xy[1], pos_z])
```

## Preview Control

### Linear Inverted Pendulum Model (LIPM)

```python
class PreviewController:
    def __init__(self, com_height=0.8):
        self.h = com_height
        self.g = 9.81
        
        # System dynamics (LIPM)
        # ẍ = (g/h) * (x - zmp)
        self.A = np.array([[0, 1, 0],
                          [0, 0, self.g/self.h],
                          [0, 0, 0]])
        self.B = np.array([[0], [0], [1]])
        self.C = np.array([[1, 0, -self.h/self.g]])
        
        # Preview window
        self.preview_steps = 100
        
        # Compute LQR gain
        self.K = self.compute_preview_gain()
    
    def compute_preview_gain(self):
        """Compute preview control gain using Riccati equation"""
        from scipy.linalg import solve_continuous_are, inv
        
        # LQR weights
        Q = np.diag([1, 0, 0])
        R = np.array([[1e-6]])
        
        # Solve Riccati equation
        P = solve_continuous_are(self.A.T, self.C.T, Q, R)
        
        # Compute gains
        K_x = inv(R + self.B.T @ P @ self.B) @ self.B.T @ P @ self.A
        K_zmp = inv(R + self.B.T @ P @ self.B) @ self.B.T @ P @ self.C.T
        
        return K_x, K_zmp
    
    def compute_control(self, current_state, zmp_reference_sequence):
        """
        current_state: [x, ẋ, ẍ]
        zmp_reference_sequence: future ZMP reference (preview window)
        """
        K_x, K_zmp = self.K
        
        # Feedback term
        u_fb = -K_x @ current_state
        
        # Feedforward term (preview)
        u_ff = 0
        for i, zmp_ref in enumerate(zmp_reference_sequence):
            u_ff += K_zmp * (zmp_ref - self.C @ current_state)
        
        # Total control (ZMP reference)
        zmp_des = u_fb + u_ff
        
        return zmp_des
```

## Model Predictive Control (MPC)

### MPC for Walking

```python
from scipy.optimize import minimize

class WalkingMPC:
    def __init__(self, horizon=20, dt=0.05):
        self.horizon = horizon
        self.dt = dt
        self.h = 0.8  # CoM height
        self.g = 9.81
    
    def solve(self, current_state, footstep_plan):
        """
        Solve MPC optimization problem
        States: [com_x, com_y, vel_x, vel_y]
        Controls: [zmp_x, zmp_y]
        """
        n_states = 4
        n_controls = 2
        
        # Initial guess
        x0 = np.zeros((self.horizon + 1) * n_states + self.horizon * n_controls)
        
        def objective(z):
            """Minimize tracking error + control effort"""
            cost = 0
            
            for k in range(self.horizon):
                # Extract state and control
                state_idx = k * n_states
                state = z[state_idx:state_idx + n_states]
                
                control_idx = (self.horizon + 1) * n_states + k * n_controls
                control = z[control_idx:control_idx + n_controls]
                
                # Tracking cost
                ref_com = self.get_reference_com(k, footstep_plan)
                cost += 100 * np.linalg.norm(state[:2] - ref_com)**2
                
                # Control effort
                cost += 0.01 * np.linalg.norm(control)**2
            
            return cost
        
        def dynamics_constraints(z):
            """Enforce system dynamics"""
            constraints = []
            
            for k in range(self.horizon):
                state_idx = k * n_states
                next_state_idx = (k + 1) * n_states
                control_idx = (self.horizon + 1) * n_states + k * n_controls
                
                state = z[state_idx:state_idx + n_states]
                next_state = z[next_state_idx:next_state_idx + n_states]
                control = z[control_idx:control_idx + n_controls]
                
                # LIPM dynamics: ẍ = (g/h)(x - zmp)
                predicted_next = state + self.dt * np.array([
                    state[2],  # dx = vx
                    state[3],  # dy = vy
                    self.g/self.h * (state[0] - control[0]),  # dvx
                    self.g/self.h * (state[1] - control[1])   # dvy
                ])
                
                # Constraint: next_state must equal predicted
                constraints.extend(predicted_next - next_state)
            
            return np.array(constraints)
        
        # ZMP must be in support polygon
        def stability_constraints(z):
            constraints = []
            for k in range(self.horizon):
                control_idx = (self.horizon + 1) * n_states + k * n_controls
                zmp = z[control_idx:control_idx + n_controls]
                
                # Get support polygon for this timestep
                polygon = self.get_support_polygon(k, footstep_plan)
                
                # Check if ZMP inside polygon (simplified)
                # ... add polygon constraints
            
            return np.array(constraints)
        
        # Solve optimization
        constraints = [
            {'type': 'eq', 'fun': dynamics_constraints},
            {'type': 'ineq', 'fun': stability_constraints}
        ]
        
        result = minimize(objective, x0, method='SLSQP', constraints=constraints)
        
        # Extract first control
        control_idx = (self.horizon + 1) * n_states
        optimal_zmp = result.x[control_idx:control_idx + n_controls]
        
        return optimal_zmp
```

## Push Recovery

### Capture Point Control

```python
class PushRecoveryController:
    def __init__(self):
        self.omega = np.sqrt(9.81 / 0.8)  # Inverted pendulum frequency
    
    def compute_capture_point(self, com_pos, com_vel):
        """
        Capture point: where to step to stop
        CP = CoM + CoM_vel / omega
        """
        cp = com_pos + com_vel / self.omega
        return cp
    
    def compute_recovery_step(self, com_pos, com_vel, push_force):
        """Compute emergency step location"""
        # Current capture point
        cp_current = self.compute_capture_point(com_pos, com_vel)
        
        # Add disturbance compensation
        cp_disturbed = cp_current + push_force / (self.robot.mass * self.omega)
        
        # Step location = capture point
        step_location = cp_disturbed[:2]  # x, y only
        
        return step_location
    
    def detect_fall(self, current_state):
        """Detect if robot is falling"""
        com_pos = current_state['com_pos']
        com_vel = current_state['com_vel']
        support_polygon = current_state['support_polygon']
        
        cp = self.compute_capture_point(com_pos, com_vel)
        
        # If capture point outside support polygon, robot is falling
        from shapely.geometry import Point, Polygon
        
        cp_point = Point(cp[0], cp[1])
        polygon = Polygon(support_polygon)
        
        return not polygon.contains(cp_point)
```

## Summary

✅ ZMP theory and stability  
✅ Gait generation  
✅ Preview control (LIPM)  
✅ Model Predictive Control  
✅ Push recovery  

## Practice Exercises

1. Implement ZMP calculator
2. Generate walking trajectory
3. Implement preview controller
4. Add push recovery

## Next Chapter

In **Chapter 16**, we'll explore:
- Manipulation and grasping
- Grasp planning
- Force control
- Dexterous manipulation

👉 [Continue to Chapter 16 →](./chapter-16.md)
