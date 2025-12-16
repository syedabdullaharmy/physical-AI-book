---
sidebar_position: 16
---

# Chapter 16: Manipulation and Grasping

## Introduction

Dexterous manipulation enables humanoids to interact with objects and perform complex tasks. This chapter covers grasp planning, force control, and coordinated manipulation.

:::tip Learning Objectives
- Plan grasps for objects
- Implement force control
- Use admittance/impedance control
- Perform bimanual manipulation
- Integrate vision for grasping
:::

## Grasp Planning

### Grasp Quality Metrics

```python
import numpy as np

class GraspPlanner:
    def __init__(self, gripper):
        self.gripper = gripper
    
    def evaluate_grasp(self, contact_points, contact_normals, friction_coeff=0.5):
        """
        Evaluate grasp quality using force closure
        """
        n_contacts = len(contact_points)
        
        # Build grasp matrix G (6 x 3n)
        G = np.zeros((6, 3*n_contacts))
        
        for i, (point, normal) in enumerate(zip(contact_points, contact_normals)):
            # Force component
            G[:3, 3*i:3*i+3] = np.eye(3)
            
            # Torque component (r × f)
            G[3:, 3*i:3*i+3] = self.skew_matrix(point)
        
        # Check force closure (rank of G should be 6)
        rank = np.linalg.matrix_rank(G)
        force_closure = (rank == 6)
        
        # Compute grasp quality (minimum singular value)
        if force_closure:
            singular_values = np.linalg.svd(G, compute_uv=False)
            quality = np.min(singular_values)
        else:
            quality = 0
        
        return quality, force_closure
    
    @staticmethod
    def skew_matrix(v):
        """Create skew-symmetric matrix from vector"""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
    
    def sample_grasps(self, object_mesh, num_samples=1000):
        """Sample and rank potential grasps"""
        grasps = []
        
        for _ in range(num_samples):
            # Sample contact points on mesh
            contacts = self.sample_contacts(object_mesh, n_contacts=4)
            
            # Compute contact normals
            normals = [object_mesh.get_normal_at(c) for c in contacts]
            
            # Evaluate grasp
            quality, valid = self.evaluate_grasp(contacts, normals)
            
            if valid:
                grasps.append({
                    'contacts': contacts,
                    'normals': normals,
                    'quality': quality
                })
        
        # Sort by quality
        grasps.sort(key=lambda g: g['quality'], reverse=True)
        
        return grasps
```

### Antipodal Grasp Generation

```python
class AntipodalGraspGenerator:
    def __init__(self, gripper_width=0.12):
        self.gripper_width = gripper_width
    
    def generate_antipodal_grasps(self, point_cloud):
        """
        Generate parallel jaw grasps from point cloud
        """
        grasps = []
        
        # Sample grasp centers
        for i in range(0, len(point_cloud), 10):
            center = point_cloud[i]
            
            # Sample approach directions
            for theta in np.linspace(0, 2*np.pi, 16):
                for phi in np.linspace(0, np.pi, 8):
                    approach = np.array([
                        np.sin(phi) * np.cos(theta),
                        np.sin(phi) * np.sin(theta),
                        np.cos(phi)
                    ])
                    
                    # Check if gripper can close
                    if self.check_collision_free(center, approach, point_cloud):
                        # Compute grasp pose
                        grasp_pose = self.compute_grasp_pose(center, approach)
                        
                        grasps.append({
                            'pose': grasp_pose,
                            'center': center,
                            'approach': approach,
                            'width': self.estimate_width(center, approach, point_cloud)
                        })
        
        return grasps
    
    def compute_grasp_pose(self, center, approach):
        """Convert center and approach to SE(3) pose"""
        # Z-axis aligned with approach
        z_axis = approach / np.linalg.norm(approach)
        
        # X-axis perpendicular to approach
        x_axis = np.cross([0, 0, 1], z_axis)
        if np.linalg.norm(x_axis) < 0.1:
            x_axis = np.cross([1, 0, 0], z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Y-axis completes right-hand frame
        y_axis = np.cross(z_axis, x_axis)
        
        # Rotation matrix
        R = np.column_stack([x_axis, y_axis, z_axis])
        
        # Full pose
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = center
        
        return T
```

## Force Control

### Hybrid Position/Force Control

```python
class HybridController:
    def __init__(self, robot):
        self.robot = robot
        self.Kp_pos = 1000  # Position stiffness
        self.Kp_force = 0.1  # Force gain
    
    def compute_control(self, current_state, desired_state, selection_matrix):
        """
        selection_matrix: 6x6 diagonal matrix
        - 1 for force control directions
        - 0 for position control directions
        """
        S = selection_matrix
        Sbar = np.eye(6) - S  # Complement
        
        # Current and desired states
        x_current = current_state['position']
        f_current = current_state['force']
        
        x_desired = desired_state['position']
        f_desired = desired_state['force']
        
        # Position error
        e_pos = x_desired - x_current
        
        # Force error
        e_force = f_desired - f_current
        
        # Hybrid control law
        tau = self.robot.jacobian.T @ (
            Sbar @ (self.Kp_pos * e_pos) +  # Position control
            S @ (self.Kp_force * e_force)    # Force control
        )
        
        return tau
```

### Impedance Control

```python
class ImpedanceController:
    def __init__(self, robot):
        self.robot = robot
        
        # Desired impedance parameters
        self.M_des = 1.0 * np.eye(6)   # Desired mass
        self.B_des = 20.0 * np.eye(6)  # Desired damping
        self.K_des = 100.0 * np.eye(6) # Desired stiffness
    
    def compute_control(self, current_state, desired_traj):
        """
        Impedance control: makes end-effector behave as mass-spring-damper
        """
        # Current state
        x = current_state['position']
        dx = current_state['velocity']
        
        # Desired trajectory
        x_des = desired_traj['position']
        dx_des = desired_traj['velocity']
        ddx_des = desired_traj['acceleration']
        
        # External force measurement
        f_ext = current_state['external_force']
        
        # Impedance equation
        ddx = ddx_des + np.linalg.inv(self.M_des) @ (
            f_ext - self.B_des @ (dx - dx_des) - self.K_des @ (x - x_des)
        )
        
        # Convert to joint torques
        J = self.robot.jacobian(current_state['joint_positions'])
        M = self.robot.mass_matrix(current_state['joint_positions'])
        
        # Operational space dynamics
        Lambda = np.linalg.inv(J @ np.linalg.inv(M) @ J.T)
        
        # Control  torque
        tau = J.T @ (Lambda @ ddx)
        
        return tau
```

### Admittance Control

```python
class AdmittanceController:
    def __init__(self):
        # Admittance parameters
        self.M = 2.0    # Virtual mass
        self.B = 10.0   # Virtual damping
        self.K = 50.0   # Virtual stiffness
    
    def compute_trajectory_modification(self, f_ext, x_des, dt):
        """
        Modify desired trajectory based on external forces
        """
        # Admittance dynamics: M*ẍ + B*ẋ + K*x = f_ext
        # In discrete time:
        x_err = 0
        dx_err = 0
        
        ddx = (f_ext - self.B * dx_err - self.K * x_err) / self.M
        dx_err += ddx * dt
        x_err += dx_err * dt
        
        # Modified desired position
        x_modified = x_des + x_err
        
        return x_modified
```

## Bimanual Manipulation

### Coordinated Dual-Arm Control

```python
class BimanualController:
    def __init__(self, left_arm, right_arm):
        self.left_arm = left_arm
        self.right_arm = right_arm
    
    def compute_relative_pose_control(self, object_pose_desired):
        """
        Control object pose using both arms
        """
        # Get current end-effector poses
        T_left = self.left_arm.forward_kinematics()
        T_right = self.right_arm.forward_kinematics()
        
        # Current object pose (center between hands)
        T_object_current = self.compute_object_pose(T_left, T_right)
        
        # Pose error
        pose_error = self.pose_difference(object_pose_desired, T_object_current)
        
        # Split error between arms
        left_target = self.distribute_left(pose_error)
        right_target = self.distribute_right(pose_error)
        
        # Compute arm controls
        tau_left = self.left_arm.compute_control(left_target)
        tau_right = self.right_arm.compute_control(right_target)
        
        return tau_left, tau_right
    
    def compute_internal_force_control(self, desired_squeeze_force):
        """Control grasping force between hands"""
        # Measure current forces
        f_left = self.left_arm.get_end_effector_force()
        f_right = self.right_arm.get_end_effector_force()
        
        # Internal force (should be equal and opposite)
        internal_force = (f_left - f_right) / 2
        
        # Force error
        force_error = desired_squeeze_force - np.linalg.norm(internal_force)
        
        # Adjust gripper forces
        adjustment = 0.1 * force_error
        
        tau_left = self.left_arm.force_control(desired_squeeze_force + adjustment)
        tau_right = self.right_arm.force_control(desired_squeeze_force + adjustment)
        
        return tau_left, tau_right
```

## Vision-Guided Grasping

### Visual Servoing

```python
class VisualServoingController:
    def __init__(self, camera, robot):
        self.camera = camera
        self.robot = robot
        self.lambda_gain = 0.5
    
    def compute_image_jacobian(self, features_3d, camera_pose):
        """
        Compute image Jacobian L
        Maps end-effector velocity to image feature velocity
        """
        L = []
        
        for point_3d in features_3d:
            x, y, z = point_3d
            
            # Image Jacobian for point feature
            L_point = np.array([
                [-1/z, 0, x/z, x*y, -(1+x**2), y],
                [0, -1/z, y/z, 1+y**2, -x*y, -x]
            ])
            
            L.append(L_point)
        
        return np.vstack(L)
    
    def compute_control(self, current_features, desired_features):
        """PBVS (Position-Based Visual Servoing)"""
        # Feature error
        e = desired_features - current_features
        
        # Image Jacobian
        L = self.compute_image_jacobian(current_features, self.camera.pose)
        
        # Camera velocity
        v_camera = -self.lambda_gain * np.linalg.pinv(L) @ e
        
        # Convert to end-effector velocity
        v_ee = self.camera_to_ee_velocity(v_camera)
        
        # Convert to joint velocities
        J = self.robot.jacobian()
        q_dot = np.linalg.pinv(J) @ v_ee
        
        return q_dot
```

## Summary

✅ Grasp planning and quality metrics  
✅ Force control (hybrid, impedance, admittance)  
✅ Bimanual manipulation  
✅ Visual servoing  

## Practice Exercises

1. Implement grasp quality evaluator
2. Create impedance controller
3. Coordinate dual-arm motion
4. Add visual servoing

## Next Chapter

In **Chapter 17**, we'll explore:
- Conversational robotics
- Natural language understanding
- Voice-to-action systems
- GPT integration

👉 [Continue to Chapter 17 →](./chapter-17.md)
