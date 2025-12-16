---
sidebar_position: 14
---

import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="module-4/chapter-14" 
    chapterTitle="Humanoid Kinematics and Dynamics" 
/>

# Chapter 14: Humanoid Kinematics and Dynamics

## Introduction

Understanding humanoid kinematics and dynamics is essential for motion planning and control. This chapter covers the mathematical foundations for computing positions, velocities, and forces in humanoid robots.

:::tip Learning Objectives
- Compute forward and inverse kinematics
- Solve 6-DOF inverse kinematics
- Calculate Jacobians for velocity control
- Understand dynamics equations (Lagrangian)
- Implement whole-body dynamics
- Use optimization for IK
:::

## Forward Kinematics

### Denavit-Hartenberg Convention

```python
import numpy as np

def dh_transform(theta, d, a, alpha):
    """
    Create DH transformation matrix
    theta: joint angle
    d: link offset
    a: link length
    alpha: link twist
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])
    
    return T

class HumanoidKinematics:
    def __init__(self):
        # DH parameters for 7-DOF arm
        # [theta, d, a, alpha]
        self.dh_params = [
            [0, 0.333,  0,     -np.pi/2],  # Shoulder 1
            [0, 0,      0,      np.pi/2],  # Shoulder 2
            [0, 0.316,  0,      np.pi/2],  # Shoulder 3
            [0, 0,      0.0825,-np.pi/2],  # Elbow
            [0, 0.384,  0,      np.pi/2],  # Wrist 1
            [0, 0,      0,     -np.pi/2],  # Wrist 2
            [0, 0.107,  0.088,  0      ],  # Wrist 3
        ]
    
    def forward_kinematics(self, joint_angles):
        """
        Compute end-effector pose from joint angles
        Returns: 4x4 transformation matrix
        """
        T = np.eye(4)
        
        for i, (theta_offset, d, a, alpha) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            T_i = dh_transform(theta, d, a, alpha)
            T = T @ T_i
        
        return T
    
    def get_position(self, joint_angles):
        """Extract position from FK"""
        T = self.forward_kinematics(joint_angles)
        return T[:3, 3]
    
    def get_orientation(self, joint_angles):
        """Extract rotation matrix from FK"""
        T = self.forward_kinematics(joint_angles)
        return T[:3, :3]
```

### Multi-Chain Forward Kinematics

```python
class HumanoidFullKinematics:
    def __init__(self):
        self.chains = {
            'left_arm': LeftArmKinematics(),
            'right_arm': RightArmKinematics(),
            'left_leg': LeftLegKinematics(),
            'right_leg': RightLegKinematics(),
            'head': HeadKinematics()
        }
    
    def compute_all_fk(self, joint_positions):
        """Compute FK for all chains"""
        end_effector_poses = {}
        
        # Left arm
        left_arm_joints = joint_positions[0:7]
        end_effector_poses['left_hand'] = \
            self.chains['left_arm'].forward_kinematics(left_arm_joints)
        
        # Right arm
        right_arm_joints = joint_positions[7:14]
        end_effector_poses['right_hand'] = \
            self.chains['right_arm'].forward_kinematics(right_arm_joints)
        
        # Legs
        left_leg_joints = joint_positions[14:18]
        end_effector_poses['left_foot'] = \
            self.chains['left_leg'].forward_kinematics(left_leg_joints)
        
        right_leg_joints = joint_positions[18:22]
        end_effector_poses['right_foot'] = \
            self.chains['right_leg'].forward_kinematics(right_leg_joints)
        
        return end_effector_poses
```

## Inverse Kinematics

### Analytical IK (for specific geometries)

```python
def inverse_kinematics_analytical(target_pos, target_rot):
    """
    Analytical IK for 6-DOF manipulator
    Only works for specific robot geometries
    """
    x, y, z = target_pos
    
    # Solve for base rotation
    theta1 = np.arctan2(y, x)
    
    # Solve for elbow
    r = np.sqrt(x**2 + y**2)
    s = z - d1
    D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
    theta3 = np.arctan2(np.sqrt(1 - D**2), D)  # Elbow up
    
    # Solve for shoulder
    alpha = np.arctan2(s, r)
    beta = np.arctan2(a3*np.sin(theta3), a2 + a3*np.cos(theta3))
    theta2 = alpha - beta
    
    # Solve for wrist (from rotation matrix)
    R_03 = dh_transform(theta1, 0, 0, -np.pi/2) @ \
           dh_transform(theta2, 0, a2, 0) @ \
           dh_transform(theta3, 0, a3, 0)
    
    R_36 = R_03[:3, :3].T @ target_rot
    
    theta4 = np.arctan2(R_36[1, 2], R_36[0, 2])
    theta5 = np.arctan2(np.sqrt(R_36[0, 2]**2 + R_36[1, 2]**2), R_36[2, 2])
    theta6 = np.arctan2(R_36[2, 1], -R_36[2, 0])
    
    return np.array([theta1, theta2, theta3, theta4, theta5, theta6])
```

### Numerical IK (Jacobian-based)

```python
class JacobianIK:
    def __init__(self, robot):
        self.robot = robot
        self.epsilon = 1e-4  # For numerical derivative
    
    def compute_jacobian(self, joint_angles):
        """
        Compute 6x7 Jacobian matrix
        J = [dpos/dq1, dpos/dq2, ..., dpos/dq7]
            [drot/dq1, drot/dq2, ..., drot/dq7]
        """
        n_joints = len(joint_angles)
        J = np.zeros((6, n_joints))
        
        # Current pose
        T_current = self.robot.forward_kinematics(joint_angles)
        pos_current = T_current[:3, 3]
        
        for i in range(n_joints):
            # Perturb joint i
            q_perturbed = joint_angles.copy()
            q_perturbed[i] += self.epsilon
            
            # Compute perturbed pose
            T_perturbed = self.robot.forward_kinematics(q_perturbed)
            pos_perturbed = T_perturbed[:3, 3]
            
            # Linear velocity Jacobian (numerical derivative)
            J[:3, i] = (pos_perturbed - pos_current) / self.epsilon
            
            # Angular velocity Jacobian (from joint axis)
            # For revolute joints, angular velocity = joint axis
            joint_axis = self.get_joint_axis(joint_angles, i)
            J[3:, i] = joint_axis
        
        return J
    
    def solve_ik(self, target_pos, target_rot, initial_guess=None, 
                 max_iter=100, tolerance=1e-3):
        """
        Solve IK using Jacobian pseudoinverse method
        """
        if initial_guess is None:
            q = np.zeros(7)
        else:
            q = initial_guess.copy()
        
        for iteration in range(max_iter):
            # Current pose
            T_current = self.robot.forward_kinematics(q)
            pos_current = T_current[:3, 3]
            rot_current = T_current[:3, :3]
            
            # Position error
            pos_error = target_pos - pos_current
            
            # Orientation error (angle-axis)
            rot_error_matrix = target_rot @ rot_current.T
            rot_error = self.rotation_matrix_to_axis_angle(rot_error_matrix)
            
            # Combined error
            error = np.concatenate([pos_error, rot_error])
            
            # Check convergence
            if np.linalg.norm(error) < tolerance:
                return q, True
            
            # Compute Jacobian
            J = self.compute_jacobian(q)
            
            # Pseudoinverse
            J_pinv = np.linalg.pinv(J)
            
            # Update (damped least squares for stability)
            lambda_damping = 0.1
            dq = J_pinv @ error
            q += 0.1 * dq  # Step size
        
        return q, False
    
    def rotation_matrix_to_axis_angle(self, R):
        """Convert rotation matrix to axis-angle representation"""
        angle = np.arccos((np.trace(R) - 1) / 2)
        if abs(angle) < 1e-6:
            return np.zeros(3)
        
        axis = 1 / (2 * np.sin(angle)) * np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ])
        
        return angle * axis
```

### Optimization-based IK

```python
from scipy.optimize import minimize

class OptimizationIK:
    def __init__(self, robot):
        self.robot = robot
    
    def solve(self, target_pos, target_rot, initial_guess=None):
        """
        Solve IK using nonlinear optimization
        """
        if initial_guess is None:
            x0 = np.zeros(7)
        else:
            x0 = initial_guess
        
        def objective(q):
            """Minimize pose error"""
            T = self.robot.forward_kinematics(q)
            pos = T[:3, 3]
            rot = T[:3, :3]
            
            # Position error
            pos_error = np.linalg.norm(target_pos - pos)**2
            
            # Orientation error
            rot_error_matrix = target_rot @ rot.T
            rot_error = np.linalg.norm(
                self.rotation_matrix_to_axis_angle(rot_error_matrix)
            )**2
            
            return pos_error + rot_error
        
        # Joint limits
        bounds = [(-2.8973, 2.8973)] * 7  # Example bounds
        
        # Optimize
        result = minimize(
            objective,
            x0,
            method='SLSQP',
            bounds=bounds
        )
        
        return result.x, result.success
```

## Dynamics

### Lagrangian Mechanics

```python
class HumanoidDynamics:
    def __init__(self, robot):
        self.robot = robot
    
    def mass_matrix(self, q):
        """
        Compute mass matrix M(q)
        τ = M(q)q̈ + C(q,q̇) + G(q)
        """
        n = len(q)
        M = np.zeros((n, n))
        
        # Composite rigid body algorithm
        for i in range(n):
            for j in range(i, n):
                # Compute M[i,j] using chain rule
                M[i, j] = self.compute_mass_matrix_element(q, i, j)
                M[j, i] = M[i, j]  # Symmetric
        
        return M
    
    def coriolis_matrix(self, q, qd):
        """
        Compute Coriolis and centrifugal forces C(q,q̇)
        """
        n = len(q)
        C = np.zeros(n)
        
        # Christoffel symbols method
        M = self.mass_matrix(q)
        
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    # Christoffel symbol
                    dM_ij_dqk = self.derivative_mass_matrix(q, i, j, k)
                    dM_ik_dqj = self.derivative_mass_matrix(q, i, k, j)
                    dM_jk_dqi = self.derivative_mass_matrix(q, j, k, i)
                    
                    c_ijk = 0.5 * (dM_ik_dqj + dM_jk_dqi - dM_ij_dqk)
                    
                    C[k] += c_ijk * qd[i] * qd[j]
        
        return C
    
    def gravity_vector(self, q):
        """Compute gravity forces G(q)"""
        n = len(q)
        G = np.zeros(n)
        
        # Total potential energy
        def potential_energy(q_var):
            total_pe = 0
            for link in self.robot.links:
                com_pos = self.robot.get_com_position(q_var, link.id)
                total_pe += link.mass * 9.81 * com_pos[2]  # z-component
            return total_pe
        
        # Gradient of potential energy
        epsilon = 1e-6
        for i in range(n):
            q_plus = q.copy()
            q_plus[i] += epsilon
            q_minus = q.copy()
            q_minus[i] -= epsilon
            
            G[i] = (potential_energy(q_plus) - potential_energy(q_minus)) / (2*epsilon)
        
        return G
    
    def forward_dynamics(self, q, qd, tau):
        """
        Solve for accelerations: q̈ = M⁻¹(τ - C - G)
        """
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        
        qdd = np.linalg.solve(M, tau - C - G)
        
        return qdd
    
    def inverse_dynamics(self, q, qd, qdd):
        """
        Compute required torques: τ = M(q)q̈ + C(q,q̇) + G(q)
        """
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        
        tau = M @ qdd + C + G
        
        return tau
```

## Whole-Body Dynamics

### Task-Space Control

```python
class WholeBodyController:
    def __init__(self, robot):
        self.robot = robot
    
    def compute_control(self, q, qd, tasks):
        """
        tasks: list of (jacobian, desired_acc, weight)
        """
        M = self.robot.mass_matrix(q)
        C = self.robot.coriolis_matrix(q, qd)
        G = self.robot.gravity_vector(q)
        
        # Stack all task Jacobians
        J_stack = []
        acc_stack = []
        W_stack = []
        
        for J, acc_des, weight in tasks:
            J_stack.append(J)
            acc_stack.append(acc_des)
            W_stack.append(weight * np.eye(len(acc_des)))
        
        J_all = np.vstack(J_stack)
        acc_all = np.concatenate(acc_stack)
        W = block_diag(*W_stack)
        
        # Solve weighted least squares:
        # min ||J*qdd - acc_des||²_W
        # subject to: τ = M*qdd + C + G
        
        A = J_all.T @ W @ J_all
        b = J_all.T @ W @ acc_all
        
        qdd = np.linalg.solve(A, b)
        tau = M @ qdd + C + G
        
        return tau
```

## Summary

✅ Forward kinematics (DH, multiple chains)  
✅ Inverse kinematics (analytical, Jacobian, optimization)  
✅ Dynamics (Lagrangian, mass matrix, Coriolis)  
✅ Whole-body control  

## Practice Exercises

1. Implement FK for your humanoid
2. Solve IK for hand reaching task
3. Compute mass matrix numerically
4. Implement gravity compensation

## Next Chapter

In **Chapter 15**, we'll explore:
- Bipedal locomotion
- ZMP-based walking
- Gait generation
- Balance control

👉 [Continue to Chapter 15 →](./chapter-15.md)
