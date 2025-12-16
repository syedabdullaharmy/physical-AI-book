---
sidebar_position: 10
---

# Chapter 10: NVIDIA Isaac SDK and Sim

## Introduction

**NVIDIA Isaac** is a comprehensive platform for AI-powered robotics, combining photorealistic simulation (Isaac Sim) with perception and navigation tools (Isaac SDK). Built on Omniverse, it leverages RTX GPUs for real-time ray tracing and physics.

:::tip Learning Objectives
- Set up Isaac Sim and Isaac SDK
- Create photorealistic robot simulations
- Use USD format for scene description
- Generate synthetic training data
- Integrate with ROS 2
- Leverage GPU acceleration
:::

## Isaac Platform Overview

```
┌─────────────────────────────────────────┐
│         NVIDIA Isaac Platform            │
├─────────────────────────────────────────┤
│  Isaac Sim (Omniverse-based)            │
│  - Photorealistic rendering (RTX)       │
│  - PhysX physics engine                 │
│  - Synthetic data generation            │
├─────────────────────────────────────────┤
│  Isaac SDK                               │
│  - Perception (object detection, SLAM)  │
│  - Navigation (Nav2 integration)        │
│  - Manipulation planning                │
├─────────────────────────────────────────┤
│  Isaac ROS (ROS 2 Packages)             │
│  - Hardware-accelerated perception      │
│  - DNN inference on Jetson              │
└─────────────────────────────────────────┘
```

## Installing Isaac Sim

### System Requirements

- **GPU:** NVIDIA RTX 2070 or higher (RTX 4080+ recommended)
- **RAM:** 32GB minimum (64GB recommended)
- **Storage:** 50GB SSD space
- **OS:** Ubuntu 20.04/22.04 or Windows 10/11
- **Drivers:** Latest NVIDIA drivers (525+)

### Installation Steps

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage

# In Launcher:
# 1. Sign in with NVIDIA account
# 2. Go to Exchange tab
# 3. Install "Isaac Sim" (latest version)
```

### Running Isaac Sim

```bash
# Launch from Omniverse Launcher
# Or from command line:
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh

# With ROS 2 bridge enabled:
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh \
    --/persistent/isaac/ros2=true
```

## USD (Universal Scene Description)

Isaac Sim uses **USD** (Pixar's Universal Scene Description) for scene files:

### Basic USD Scene

**File:** `warehouse.usd`

```python
from pxr import Usd, UsdGeom, Gf

# Create stage
stage = Usd.Stage.CreateNew('warehouse.usd')

# Add ground plane
ground = UsdGeom.Mesh.Define(stage, '/World/Ground')
ground.CreatePointsAttr([
    (-10, -10, 0), (10, -10, 0), 
    (10, 10, 0), (-10, 10, 0)
])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])

# Add box obstacle
box_xform = UsdGeom.Xform.Define(stage, '/World/Box')
box_xform.AddTranslateOp().Set(Gf.Vec3d(2, 0, 0.5))

box = UsdGeom.Cube.Define(stage, '/World/Box/Geometry')
box.CreateSizeAttr(1.0)

# Save
stage.GetRootLayer().Save()
print("Created warehouse.usd")
```

## Creating Robots in Isaac Sim

### Importing URDF

Isaac Sim can import URDF directly:

```python
import omni
from omni.isaac.core.utils.extensions import enable_extension

# Enable URDF importer
enable_extension("omni.isaac.urdf")

# Import URDF
from omni.isaac.urdf import _urdf

result, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/path/to/humanoid.urdf",
    import_config=_urdf.ImportConfig(
        set_instanceable=False,
        import_inertia_tensor=True,
        default_physics_material_path="/World/PhysicsMaterial"
    )
)

print(f"Robot imported at: {prim_path}")
```

### Programmatic Robot Creation

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Get Isaac Sim assets
assets_root = get_assets_root_path()

# Add Franka robot
franka_usd = assets_root + "/Isaac/Robots/Franka/franka.usd"
add_reference_to_stage(franka_usd, "/World/Franka")

# Create robot instance
from omni.isaac.core import World

world = World()
robot = world.scene.add(
    Robot(
        prim_path="/World/Franka",
        name="franka"
    )
)

world.reset()
```

## Physics Simulation

### PhysX Configuration

```python
from pxr import PhysxSchema, UsdPhysics

# Get stage
stage = omni.usd.get_context().get_stage()

# Create physics scene
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# PhysX-specific settings
physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
physx_scene.CreateEnableCCDAttr(True)  # Continuous collision detection
physx_scene.CreateEnableStabilizationAttr(True)
physx_scene.CreateEnableGPUDynamicsAttr(True)  # GPU acceleration
physx_scene.CreateBroadphaseTypeAttr("GPU")
physx_scene.CreateSolverTypeAttr("TGS")  # Temporal Gauss-Seidel
```

### Rigid Body Dynamics

```python
from omni.isaac.core.objects import DynamicCuboid

# Create dynamic object
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="dynamic_cube",
    position=np.array([0, 0, 2.0]),
    scale=np.array([0.5, 0.5, 0.5]),
    color=np.array([1.0, 0.0, 0.0]),
    mass=1.0
)

# Set physics material
from omni.isaac.core.materials import PhysicsMaterial

physics_mat = PhysicsMaterial(
    prim_path="/World/PhysicsMaterial",
    static_friction=0.5,
    dynamic_friction=0.4,
    restitution=0.3
)

cube.apply_physics_material(physics_mat)
```

## ROS 2 Integration

### Enabling ROS 2 Bridge

```python
import omni.graph.core as og

# Create ROS 2 graph
keys = og.Controller.Keys

(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
        ],
    },
)

print("ROS 2 bridge configured")
```

### Publishing Joint States

```python
# Add joint state publisher node
(graph, nodes, _, _) = og.Controller.edit(
    graph,
    {
        keys.CREATE_NODES: [
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        keys.SET_VALUES: [
            ("PublishJointState.inputs:targetPrim", "/World/Franka"),
            ("PublishJointState.inputs:topicName", "joint_states"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ],
    },
)
```

### Subscribing to Twist Commands

```python
# Add velocity command subscriber
(graph, nodes, _, _) = og.Controller.edit(
    graph,
    {
        keys.CREATE_NODES: [
            ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("ScaleToVelocity", "omni.isaac.core_nodes.IsaacScaleToFromVelocity"),
        ],
        keys.SET_VALUES: [
            ("SubscribeTwist.inputs:topicName", "cmd_vel"),
            ("ScaleToVelocity.inputs:targetPrim", "/World/Robot"),
        ],
        keys.CONNECT: [
            ("SubscribeTwist.outputs:linearVelocity", "ScaleToVelocity.inputs:linearVelocity"),
            ("SubscribeTwist.outputs:angularVelocity", "ScaleToVelocity.inputs:angularVelocity"),
        ],
    },
)
```

## Camera and Sensor Simulation

### RGB Camera with ROS 2 Publishing

```python
from omni.isaac.sensor import Camera
import omni.replicator.core as rep

# Create camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2.0, 2.0, 2.0]),
    frequency=30,
    resolution=(1280, 720)
)

# Create render product
render_product = rep.create.render_product(
    camera.prim_path,
    resolution=(1280, 720)
)

# Attach ROS 2 publisher
from omni.isaac.ros2_bridge import create_camera_helper

create_camera_helper(
    camera_prim_path="/World/Camera",
    topic_name="rgb/image_raw",
    publish_rgb=True,
    publish_depth=False
)
```

### Depth Camera and Point Cloud

```python
# Enable depth rendering
camera.initialize()
camera.add_distance_to_image_plane_to_frame()

# Publish depth
create_camera_helper(
    camera_prim_path="/World/Camera",
    topic_name="depth/image_raw",
    publish_rgb=False,
    publish_depth=True,
    publish_pointcloud=True
)
```

### LiDAR Simulation

```python
from omni.isaac.range_sensor import _range_sensor

# Create rotating LiDAR
result, lidar_prim = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path="/World/Lidar",
    parent="/World/Robot",
    min_range=0.4,
    max_range=100.0,
    draw_points=True,
    draw_lines=False,
    horizontal_fov=360.0,
    vertical_fov=30.0,
    horizontal_resolution=0.4,
    vertical_resolution=4.0,
    rotation_rate=20.0,  # Hz
    high_lod=False,
    yaw_offset=0.0,
    enable_semantics=False
)

# Publish to ROS 2
from omni.isaac.ros2_bridge import create_lidar_scan_publisher

create_lidar_scan_publisher(
    lidar_prim_path="/World/Lidar",
    topic_name="scan"
)
```

## Synthetic Data Generation

### Replicator for Domain Randomization

```python
import omni.replicator.core as rep

# Register randomization
with rep.new_layer():
    # Randomize lighting
    def randomize_lights():
        lights = rep.get.prims(path_pattern="/World/Lights/*")
        with lights:
            rep.modify.attribute("intensity", rep.distribution.uniform(1000, 10000))
            rep.modify.attribute("color", rep.distribution.uniform((0.5, 0.5, 0.5), (1, 1, 1)))
        return lights.node
    
    # Randomize object positions
    def randomize_objects():
        objects = rep.get.prims(path_pattern="/World/Objects/*")
        with objects:
            rep.modify.pose(
                position=rep.distribution.uniform((-2, -2, 0), (2, 2, 2)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )
        return objects.node
    
    # Randomize textures
    def randomize_materials():
        objects = rep.get.prims(semantics=[("class", "object")])
        with objects:
            rep.randomizer.materials(
                materials=rep.get.prims(path_pattern="/World/Looks/*")
            )
        return objects.node
    
    # Register randomizers
    rep.randomizer.register(randomize_lights)
    rep.randomizer.register(randomize_objects)
    rep.randomizer.register(randomize_materials)
    
    # Trigger on each frame
    with rep.trigger.on_frame():
        rep.randomizer.randomize_lights()
        rep.randomizer.randomize_objects()
        rep.randomizer.randomize_materials()
```

### Annotated Data Export

```python
# Set up writers for labeled data
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/data/synthetic",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    instance_id_segmentation=True,
    distance_to_camera=True,
    bounding_box_3d=True,
    occlusion=True
)

# Attach writer
render_product = rep.create.render_product(
    "/World/Camera",
    (1280, 720)
)
writer.attach([render_product])

# Run data generation
rep.orchestrator.run()
```

## Isaac ROS Perception

### Object Detection (DOPE)

```python
from omni.isaac.dope import DOPE

# Initialize DOPE (Deep Object Pose Estimation)
dope = DOPE()

# Configure for YCB objects
dope.set_object_models([
    "cracker_box",
    "sugar_box",
    "tomato_soup_can",
    "mustard_bottle"
])

# Enable ROS 2 publishing
dope.enable_ros2_publishing(
    topic_name="/dope/detections"
)

# Start inference
dope.start()
```

### Stereo Depth Estimation

```python
from omni.isaac.ros2_bridge import create_stereo_camera_helper

# Create stereo pair
create_stereo_camera_helper(
    left_camera_path="/World/StereoCamera/Left",
    right_camera_path="/World/StereoCamera/Right",
    baseline=0.12,  # meters
    topic_namespace="stereo"
)
```

## Performance Optimization

### GPU Acceleration

```python
# Enable all GPU features
from pxr import PhysxSchema

physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())

# GPU dynamics
physx_scene_api.CreateEnableGPUDynamicsAttr(True)
physx_scene_api.CreateGpuMaxRigidContactCountAttr(1024 * 512)
physx_scene_api.CreateGpuMaxRigidPatchCountAttr(1024 * 80)

# Broad phase on GPU
physx_scene_api.CreateBroadphaseTypeAttr("GPU")

# Solver on GPU
physx_scene_api.CreateSolverTypeAttr("TGS")
```

### Substepping

```python
# Physics substeps for stability
scene.CreateTimeStepsPerSecondAttr(60.0)
physx_scene_api.CreateSubstepCountAttr(4)
```

## Complete Humanoid Example

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Initialize world
world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)

# Import humanoid
add_reference_to_stage(
    usd_path="/path/to/humanoid.usd",
    prim_path="/World/Humanoid"
)

# Add to scene
humanoid = world.scene.add(
    Robot(
        prim_path="/World/Humanoid",
        name="humanoid_robot"
    )
)

# Reset world
world.reset()

# Control loop
for i in range(1000):
    # Set joint targets
    joint_positions = np.array([
        np.sin(i * 0.01) * 0.5,  # Joint 1
        np.cos(i * 0.01) * 0.5,  # Joint 2
        # ... more joints
    ])
    
    humanoid.set_joint_positions(joint_positions)
    
    # Step simulation
    world.step(render=True)

# Cleanup
world.stop()
```

## Summary

✅ Isaac Sim setup and USD basics  
✅ Robot import and creation  
✅ PhysX physics simulation  
✅ ROS 2 integration  
✅ Camera and LiDAR sensors  
✅ Synthetic data generation  
✅ GPU acceleration  

## Practice Exercises

1. Import your humanoid and simulate in Isaac Sim
2. Set up ROS 2 bridge for joint control
3. Generate 1000 synthetic images with randomization
4. Configure GPU-accelerated physics

## Next Chapter

In **Chapter 11**, we'll explore:
- AI-powered perception
- Object detection and tracking
- Pose estimation
- Real-time inference on Jetson

👉 [Continue to Chapter 11 →](./chapter-11.md)
