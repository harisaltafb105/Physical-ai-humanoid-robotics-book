---
id: simulation-essentials
title: 'Chapter 1: Simulation Essentials - Gazebo & Physics'
sidebar_position: 1
last_updated: 2025-12-06
---

# Chapter 1: Simulation Essentials - Gazebo & Physics

## Learning Objectives

After completing this chapter, you will be able to:

- Explain the role of physics simulation in robotics development
- Set up Gazebo simulation environments with custom worlds
- Understand physics engine fundamentals (gravity, collisions, timestep)
- Spawn robots and objects in simulation
- Configure simulation parameters for realistic behavior

**Reading time**: 12 minutes

## Why Gazebo Simulation?

### The Sim-Before-Deploy Philosophy

Imagine programming a $50,000 humanoid robot to walk up stairs. Would you:
- **Option A**: Write code, deploy to robot, watch it fall and break
- **Option B**: Test in simulation first, then deploy when confident

**Gazebo** is the industry-standard physics simulator for ROS 2, enabling Option B. It provides:

✅ **Safety**: Test dangerous scenarios (falls, collisions) without hardware damage
✅ **Speed**: Restart scenarios instantly—no physical setup needed
✅ **Scale**: Simulate 10+ robots on one workstation
✅ **Reproducibility**: Exact same conditions every run
✅ **Data generation**: Create synthetic training data for perception models

### Real-World Usage

- **Clearpath Robotics**: Tests autonomous navigation algorithms in Gazebo before warehouse deployment
- **Agility Robotics**: Simulates bipedal robot Digit climbing stairs and navigating obstacles
- **Research labs worldwide**: Develop new algorithms 100x faster with simulation

## Gazebo Architecture

### Core Components

```text
┌─────────────────────────────────────────┐
│         Gazebo Simulator                │
│  ┌──────────┐  ┌──────────┐  ┌────────┐│
│  │ Physics  │  │ Rendering│  │ Sensors││
│  │ Engine   │←─│ Engine   │←─│ Plugins││
│  │ (ODE/    │  │ (OGRE)   │  │        ││
│  │ Bullet)  │  │          │  │        ││
│  └──────────┘  └──────────┘  └────────┘│
│        ↓             ↓            ↓     │
│  Computes       Displays      Simulates│
│  forces         visuals       sensors  │
└─────────────────────────────────────────┘
         ↓ Topics/Services
┌─────────────────────────────────────────┐
│            ROS 2 Nodes                  │
│  Controllers, Planners, Perception      │
└─────────────────────────────────────────┘
```

**Physics Engine**: Computes gravity, collisions, friction (Gazebo supports ODE, Bullet, DART, Simbody)
**Rendering Engine**: Displays 3D visuals using OGRE (fast, GPU-accelerated)
**Sensor Plugins**: Generate realistic sensor data (LiDAR point clouds, camera images, IMU readings)

## Physics Simulation Basics

### Rigid Body Dynamics

Gazebo simulates robots as collections of **rigid bodies** (links) connected by **joints**:

```python
# A simple box in simulation (specified in SDF/URDF)
<link name="box">
  <inertial>
    <mass>5.0</mass>  <!-- 5 kg -->
    <inertia>         <!-- Moment of inertia (3x3 matrix) -->
      <ixx>0.1</ixx>
      <iyy>0.1</iyy>
      <izz>0.1</izz>
    </inertia>
  </inertial>
  <collision>
    <geometry>
      <box size="1.0 1.0 1.0"/>  <!-- 1m cube -->
    </geometry>
  </collision>
  <visual>
    <geometry>
      <box size="1.0 1.0 1.0"/>
    </geometry>
  </visual>
</link>
```

**Key properties**:
- **Mass**: Determines force required to accelerate (F = ma)
- **Inertia**: Resistance to rotational acceleration (important for humanoid balance!)
- **Collision geometry**: Simplified shapes for fast collision detection
- **Visual geometry**: High-detail meshes for rendering

### Gravity and Forces

By default, Gazebo applies Earth's gravity: `g = 9.81 m/s²` downward (negative z-axis).

You can modify gravity to simulate:
- **Moon**: `g = 1.62 m/s²` (test lunar rovers)
- **Mars**: `g = 3.71 m/s²`
- **Zero-g**: `g = 0` (test space robots)
- **Custom**: Any vector (e.g., sideways gravity for testing balance algorithms)

### Collision Detection

Gazebo checks for overlaps between collision geometries every timestep:

1. **Broad phase**: Quickly eliminate distant objects (using bounding boxes)
2. **Narrow phase**: Precise collision test for nearby objects
3. **Contact resolution**: Compute contact forces and friction

**Performance tip**: Use simple collision shapes (boxes, spheres, cylinders) instead of complex meshes.

### Timestep and Integration

Gazebo advances simulation in discrete steps:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1 ms timestep -->
  <real_time_factor>1.0</real_time_factor>  <!-- Run at real-time speed -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz -->
</physics>
```

**Timestep trade-offs**:
- **Smaller timestep** (e.g., 0.0001s): More accurate, slower computation
- **Larger timestep** (e.g., 0.01s): Faster, but may miss collisions or become unstable

**Rule of thumb**: For humanoid robots with fast dynamics, use `max_step_size ≤ 0.001` (1 ms).

## Hands-On: Your First Gazebo World

### Step 1: Install Gazebo Garden

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs gazebo11
```

Check installation:
```bash
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x
```

### Step 2: Launch Empty World

```bash
gazebo --verbose
```

You should see:
- Gray ground plane (infinite collision plane)
- Blue sky (just rendering, no physics)
- Camera controls (left-click drag to orbit, scroll to zoom)

### Step 3: Add a Box

**Method 1: GUI**:
1. Click "Insert" tab on the left
2. Select "Box"
3. Click anywhere in the world to place it
4. Press Spacebar to start physics—box falls and lands on ground!

**Method 2: Programmatic (SDF file)**:

Create `box_world.sdf`:
```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="box_world">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun (lighting) -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Falling box -->
    <model name="my_box">
      <pose>0 0 5 0 0 0</pose>  <!-- Start at z=5m -->
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>  <!-- Red color -->
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

Launch:
```bash
gazebo box_world.sdf
```

**Expected behavior**: Red box falls from 5m height, hits ground, settles.

### Step 4: Spawn a Robot

Download a sample URDF (e.g., TurtleBot3):
```bash
sudo apt install ros-humble-turtlebot3-gazebo
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

You'll see a small differential drive robot in Gazebo. Try sending velocity commands:
```bash
# In another terminal
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.5}}'
```

The robot moves forward and turns—ROS 2 commands controlling simulated robot!

## Debugging Simulation Issues

### Common Problems

| Problem | Cause | Solution |
|---------|-------|----------|
| Robot falls through ground | Missing collision geometry | Add `<collision>` to URDF |
| Robot explodes/vibrates | Timestep too large | Reduce `max_step_size` to 0.001 |
| Simulation runs slowly | Too many complex meshes | Use simple collision shapes |
| Robot doesn't move | Incorrect joint type | Verify `<joint type="revolute">` for wheels |

### Performance Optimization

```xml
<!-- Optimize contact computation -->
<surface>
  <contact>
    <ode>
      <soft_cfm>0.01</soft_cfm>  <!-- Contact softness -->
      <soft_erp>0.2</soft_erp>   <!-- Error reduction -->
      <kp>1e6</kp>               <!-- Contact stiffness -->
      <kd>1</kd>                 <!-- Contact damping -->
    </ode>
  </contact>
</surface>
```

## Summary

In this chapter, you learned:

✅ **Why simulate**: Safety, speed, reproducibility, data generation
✅ **Gazebo architecture**: Physics engine + rendering + sensors
✅ **Physics basics**: Rigid bodies, gravity, collisions, timestep
✅ **Hands-on skills**: Launch Gazebo, spawn models, configure physics

**Key takeaway**: Simulation is not "optional"—it's essential for modern robotics. Test in simulation first, deploy to hardware last.

## Next Steps

In [Chapter 2: Sensors & ROS 2 Integration](./02-sensors-ros2.md), you'll learn how to:
- Simulate LiDAR, cameras, and IMUs
- Bridge sensor data to ROS 2 topics
- Control simulated robots from ROS 2 nodes

## Further Reading

- [Gazebo Official Tutorials](http://gazebosim.org/tutorials)
- [SDF Format Specification](http://sdformat.org/spec)
- [Physics Engine Comparison (ODE vs Bullet)](https://classic.gazebosim.org/tutorials?tut=physics_params)

---

**Hands-on exercise**: Create a world with 3 boxes stacked like a tower. What happens when you apply a force to the bottom box? Experiment with friction settings!
