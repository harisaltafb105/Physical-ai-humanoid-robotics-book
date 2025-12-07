---
id: glossary
title: Glossary
sidebar_position: 100
last_updated: 2025-12-06
---

# Glossary

Quick reference for Physical AI and robotics terminology used throughout this book.

---

## ROS 2 Terms

### Colcon
Build tool for ROS 2 workspaces. Compiles packages and manages dependencies.

### Joint
Connection between two robot links. Defines motion constraints (revolute, prismatic, fixed, continuous).

### Message
Typed data structure passed between nodes over topics (e.g., `std_msgs/String`, `geometry_msgs/Twist`).

### Middleware
Software layer that provides communication between distributed components. ROS 2 uses DDS (Data Distribution Service).

### Node
Independent executable process performing a specific robot task. Nodes communicate via topics, services, and actions.

### Publisher
Node that sends messages to a topic.

### QoS (Quality of Service)
Configuration for message delivery reliability and performance (RELIABLE vs BEST_EFFORT).

### rclpy
ROS Client Library for Python. Provides API for creating nodes, publishers, subscribers in Python.

### Service
Request-response communication pattern. Client sends request, server processes and returns response.

### Subscriber
Node that receives messages from a topic.

### Topic
Named channel for asynchronous message passing. Enables publish-subscribe communication.

### URDF (Unified Robot Description Format)
XML-based format for describing robot geometry, kinematics, visual properties, and collision boundaries.

### Workspace
Directory containing ROS 2 packages, typically with `src/`, `build/`, `install/` subdirectories.

---

## Physical AI Terms

### LiDAR (Light Detection and Ranging)
Laser-based sensor that measures distances to create 3D maps of the environment.

### Physical AI
Artificial intelligence systems that interact with the physical world through sensors and actuators (robots).

### RViz
ROS visualization tool for displaying robot models, sensor data, and transforms in 3D.

### Simulation
Virtual environment for testing robot software before deploying to hardware.

### SLAM (Simultaneous Localization and Mapping)
Algorithm for building a map while tracking the robot's location within it.

---

## Robotics Hardware

### Actuator
Device that produces motion (motor, servo, pneumatic cylinder).

### DoF (Degrees of Freedom)
Number of independent movements a robot can make. Humanoids typically have 20-40 DoF.

### End Effector
Tool at the end of a robot arm (gripper, hand, suction cup, tool).

### IMU (Inertial Measurement Unit)
Sensor measuring acceleration and rotation using accelerometers and gyroscopes.

### Jetson (NVIDIA Jetson)
Embedded AI computing platform for edge robotics (Jetson Orin, Jetson AGX).

### RealSense (Intel RealSense)
Depth camera providing RGB-D (color + depth) data for 3D perception.

### RTX GPU
NVIDIA graphics processor with ray tracing and tensor cores for AI acceleration.

### Sensor
Device that measures physical properties (camera, LiDAR, IMU, force/torque sensor).

---

## Simulation Terms

### Gazebo
Physics-based robot simulator with realistic dynamics and sensor models.

### Isaac Sim (NVIDIA Isaac Sim)
Photorealistic robot simulator built on Omniverse, with GPU-accelerated rendering and physics.

### ODE (Open Dynamics Engine)
Physics engine used by Gazebo for collision detection and dynamics simulation.

### Synthetic Data
Artificially generated training data from simulation (images, point clouds, sensor readings).

### USD (Universal Scene Description)
File format for 3D scenes used by NVIDIA Omniverse and Isaac Sim.

---

## AI & Perception

### LLM (Large Language Model)
Neural network trained on text (GPT, Claude) that can understand and generate natural language.

### VSLAM (Visual SLAM)
SLAM using camera images instead of LiDAR for mapping and localization.

### VLA (Vision-Language-Action)
AI model that takes visual input and language commands, outputs robot actions.

### Whisper (OpenAI Whisper)
Open-source speech recognition model for voice-to-text transcription.

---

## Additional Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **NVIDIA Isaac Sim**: https://developer.nvidia.com/isaac-sim
- **Gazebo**: https://gazebosim.org/
- **ROS Answers**: https://answers.ros.org/

---

**Need a term explained?** Open a GitHub issue and we'll add it to the glossary!
