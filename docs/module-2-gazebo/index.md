---
id: module-2-gazebo
title: 'Module 2: Digital Twin - Gazebo Simulation'
sidebar_position: 2
last_updated: 2025-12-06
---

# Module 2: Digital Twin - Gazebo Simulation

## Module Overview

Welcome to Module 2! After mastering ROS 2 fundamentals in Module 1, you're ready to explore the power of **physics simulation**. In this module, you'll learn how to create **digital twins** of robots using Gazebo, enabling you to test and iterate on robot behaviors safely and rapidly—before deploying to expensive physical hardware.

**Why Simulation Matters**: Testing on physical robots is slow, risky, and costly. Simulation lets you:
- **Iterate faster**: Restart scenarios instantly, no hardware setup
- **Test dangerous scenarios**: Collisions, falls, extreme environments—without damage
- **Generate infinite data**: Train perception models with synthetic sensor data
- **Scale development**: Simulate 10+ robots simultaneously on one workstation

## Learning Objectives

After completing this module, you will be able to:

1. Set up Gazebo simulation environments with custom worlds and robot models
2. Understand physics simulation fundamentals (gravity, collisions, friction, timestep)
3. Configure virtual sensors (LiDAR, depth cameras, IMUs) to generate realistic data
4. Bridge Gazebo simulations to ROS 2 nodes for closed-loop control
5. Debug simulation issues and optimize performance

## Module Structure

This module contains 2 chapters (estimated reading time: 24 minutes):

1. **Chapter 1: Simulation Essentials** (12 minutes)
   - What is Gazebo and why use it
   - Physics simulation basics (gravity, rigid body dynamics, collisions)
   - Creating your first simulation world
   - Spawning robots and objects

2. **Chapter 2: Sensors & ROS 2 Integration** (12 minutes)
   - Simulating LiDAR, depth cameras (RealSense), and IMUs
   - Publishing sensor data to ROS 2 topics
   - Connecting simulated robots to ROS 2 control nodes
   - Practical example: Teleoperation in simulation

## Prerequisites

- **Module 1 completed**: ROS 2 fundamentals, nodes, topics, URDF
- **Gazebo Fortress or Garden** installed (compatible with ROS 2 Humble/Iron)
- **Basic 3D geometry concepts**: Understanding of coordinate frames (x, y, z)

## What You'll Build

By the end of this module, you'll have:

- A functional Gazebo simulation world with obstacles and terrain
- A simulated humanoid robot with virtual sensors (LiDAR, depth camera, IMU)
- ROS 2 nodes that read sensor data and publish motor commands to the simulated robot
- The foundation for Module 3 (NVIDIA Isaac Sim) and Module 4 (VLA integration)

## Key Concepts Introduced

- **Digital Twin**: A virtual replica of a physical robot used for testing and development
- **Physics Engine**: Software that computes gravity, collisions, and forces (Gazebo uses ODE or Bullet)
- **Sensor Simulation**: Generating realistic sensor data (point clouds, images, IMU readings) in simulation
- **Sim-to-Real Transfer**: Techniques to ensure behaviors learned in simulation work on physical robots

## Real-World Applications

- **Boston Dynamics** tests humanoid Atlas robots in simulation before attempting complex parkour
- **Waymo** trains autonomous driving systems on millions of simulated miles
- **NASA** simulates Mars rovers in realistic terrain before deployment

## Next Steps

Ready to dive in? Start with **[Chapter 1: Simulation Essentials](./01-simulation-essentials.md)** to build your first Gazebo world!

---

**Estimated time to complete**: 24 minutes (reading) + 30 minutes (hands-on exercises)
