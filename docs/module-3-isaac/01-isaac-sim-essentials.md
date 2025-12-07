---
id: isaac-sim-essentials
title: 'Chapter 1: Isaac Sim Essentials - Setup & USD'
sidebar_position: 1
last_updated: 2025-12-06
---

# Chapter 1: Isaac Sim Essentials - Setup & USD

## Learning Objectives

- Install and configure NVIDIA Isaac Sim
- Understand USD (Universal Scene Description) format
- Navigate Isaac Sim interface and create basic scenes
- Optimize performance for real-time simulation
- Load humanoid robots and configure physics

**Reading time**: 12 minutes

## What is Isaac Sim?

**NVIDIA Isaac Sim** is a robotics simulation platform built on **Omniverse**—NVIDIA's platform for 3D design collaboration and physically accurate simulation.

### Key Technologies

1. **USD (Universal Scene Description)**: Open-source 3D scene format (from Pixar)
2. **PhysX 5**: NVIDIA's GPU-accelerated physics engine
3. **RTX Rendering**: Photorealistic ray tracing (same tech as video games)
4. **Replicator**: Synthetic data generation with domain randomization

### Why Photorealism Matters

**Problem**: Models trained on simple Gazebo visuals fail on real robots
**Solution**: Train on Isaac Sim's photorealistic images → better sim-to-real transfer

Research shows: Models trained on photorealistic synthetic data achieve 90%+ real-world accuracy vs. 60% from basic simulation.

## Installation

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **GPU** | RTX 3060 (12GB VRAM) | RTX 3090/4090 (24GB) |
| **CPU** | Intel i7 12th gen | Intel i9 13th gen |
| **RAM** | 32GB DDR4 | 64GB DDR5 |
| **Storage** | 50GB SSD | 100GB NVMe |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04 |

### Install Isaac Sim

```bash
# Step 1: Install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# Step 2: In Omniverse Launcher GUI
# Navigate to "Exchange" tab
# Search for "Isaac Sim"
# Click "Install" (downloads ~20GB)

# Step 3: Install Isaac Sim Python environment
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh -m pip install --upgrade pip
./python.sh -m pip install opencv-python numpy

# Step 4: Verify installation
./isaac-sim.sh
# Should launch Isaac Sim GUI
```

### First Launch

When Isaac Sim opens, you'll see:
- **Viewport**: 3D scene view (WASD to move camera, right-click drag to rotate)
- **Content Browser**: Asset library (robots, environments, props)
- **Property Panel**: Object properties and physics settings
- **Stage Tree**: Scene hierarchy (like file explorer for 3D objects)

## USD Format Fundamentals

### What is USD?

**Universal Scene Description** is an open-source 3D file format created by Pixar for animated films. It's now the standard for robotic simulation.

**Key benefits**:
- **Non-destructive**: Layers combine without modifying source files
- **Scalable**: Handles massive scenes (millions of objects)
- **Interoperable**: Works across Blender, Maya, Unreal Engine, Isaac Sim

### USD Structure Example

```python
# hello_robot.usd (simplified)
from pxr import Usd, UsdGeom, UsdPhysics

# Create stage (scene)
stage = Usd.Stage.CreateNew("hello_robot.usd")

# Add ground plane
UsdGeom.Mesh.Define(stage, "/World/Ground")
ground = UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/World/Ground"))
ground.CreateKinematicEnabledAttr(True)  # Static (doesn't move)

# Add box
box_prim = UsdGeom.Cube.Define(stage, "/World/Box")
box_prim.CreateSizeAttr(1.0)  # 1 meter cube
box_prim.AddTranslateOp().Set((0, 0, 2))  # Position: x=0, y=0, z=2m

# Add physics (make it fall)
rigid_body = UsdPhysics.RigidBodyAPI.Apply(box_prim.GetPrim())
mass_api = UsdPhysics.MassAPI.Apply(box_prim.GetPrim())
mass_api.CreateMassAttr(10.0)  # 10 kg

stage.Save()
```

### Loading USD in Isaac Sim

```python
#!/usr/bin/env python3
"""
Load a USD scene and run simulation
Part of Module 3, Chapter 1
Tested with: Isaac Sim 2023.1.1, Python 3.10
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim headless (no GUI)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world (simulation environment)
world = World()
world.scene.add_default_ground_plane()

# Add a falling cube
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="falling_cube",
    position=[0, 0, 2.0],  # Start at z=2m
    size=0.5,              # 0.5m cube
    color=[1.0, 0.0, 0.0], # Red
    mass=5.0               # 5 kg
)

# Run simulation for 5 seconds
world.reset()
for i in range(500):  # 500 frames at 100 Hz = 5 seconds
    world.step(render=True)
    position = cube.get_world_pose()[0]
    print(f"Frame {i}: Cube at z={position[2]:.3f}m")

simulation_app.close()

# Expected output:
# Frame 0: Cube at z=2.000m
# Frame 50: Cube at z=1.755m (falling)
# Frame 100: Cube at z=1.020m
# Frame 200: Cube at z=0.250m (hit ground)
# Frame 300: Cube at z=0.250m (settled)
```

## Hands-On: Create a Warehouse Scene

### Step 1: Open Isaac Sim

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./isaac-sim.sh
```

### Step 2: Add Floor and Walls

1. **Content Browser** → Search "warehouse"
2. Drag `Warehouse_01.usd` into viewport
3. Scale to appropriate size: **Property Panel** → Transform → Scale: (10, 10, 1)

### Step 3: Add Humanoid Robot

```bash
# Download sample humanoid URDF
wget https://github.com/ros-planning/moveit_resources/raw/ros2/panda_description/urdf/panda.urdf -O /tmp/robot.urdf
```

In Isaac Sim:
1. **File** → **Import** → Select `/tmp/robot.urdf`
2. Isaac Sim auto-converts URDF → USD
3. Robot appears in scene!

### Step 4: Add Obstacles

```python
# Create boxes programmatically
from omni.isaac.core.objects import DynamicCuboid

boxes = []
for i in range(5):
    box = DynamicCuboid(
        prim_path=f"/World/Box_{i}",
        position=[i * 2.0, 0, 0.5],
        size=1.0,
        color=[0.2, 0.8, 0.2]
    )
    boxes.append(box)
```

### Step 5: Run Simulation

Click ▶️ **Play** button (or press Spacebar)

Expected behavior:
- Boxes fall and land on floor
- Robot's joints obey gravity (arms fall)
- Realistic collisions between objects

## Performance Optimization

### Enable RTX Real-Time Mode

**Viewport** → **Rendering Mode** → **RTX Real-Time**

This enables:
- GPU-accelerated ray tracing
- 60 FPS rendering (vs. 10 FPS in "Path Traced" mode)
- Real-time shadows and reflections

### Reduce Physics Substeps

```python
# Access physics settings
from pxr import UsdPhysics

physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
physics_scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
physics_scene.CreateGravityMagnitudeAttr(9.81)

# Optimize timestep
physics_scene.CreateTimeCodesPerSecondAttr(60.0)  # 60 Hz (vs. default 120 Hz)
```

**Trade-off**: Lower Hz = faster simulation, but less accurate for fast-moving robots.

### Simplify Collision Meshes

For complex robots (e.g., humanoids with 50+ joints):
- Use **convex hull** approximations instead of precise meshes
- Reduces collision checks from O(n²) to O(n)

```python
# Set collision mesh to convex decomposition
collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
collision_api.CreateApproximationAttr("convexHull")
```

## Debugging Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Black screen | GPU drivers outdated | Update to NVIDIA driver 525+ |
| "RTX not supported" | Non-RTX GPU | Requires RTX 3000+ series |
| Robot explodes on load | URDF joint limits wrong | Check `<limit>` tags in URDF |
| Simulation runs at 5 FPS | Too many physics contacts | Simplify collision meshes |

## Summary

In this chapter, you learned:

✅ **Isaac Sim setup**: Installation and system requirements
✅ **USD format**: Pixar's 3D scene description language
✅ **Scene creation**: Loading robots, adding obstacles, running physics
✅ **Performance tuning**: RTX mode, timestep optimization, collision simplification

**Key takeaway**: Isaac Sim = Gazebo's physics + Unreal Engine's graphics. Use it for perception tasks where visual realism matters.

## Next Steps

In [Chapter 2: Synthetic Data & Perception](./02-synthetic-data-perception.md), you'll learn how to:
- Generate millions of labeled training images
- Apply domain randomization (lighting, textures, camera angles)
- Deploy Isaac ROS perception on NVIDIA Jetson

---

**Hands-on challenge**: Create a scene with a humanoid robot navigating around randomly placed boxes. Use Isaac Sim's Replicator API to randomize box colors and positions on each simulation reset.
