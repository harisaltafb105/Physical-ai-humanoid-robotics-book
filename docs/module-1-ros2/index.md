---
id: index
title: 'Module 1: ROS 2 Fundamentals'
sidebar_position: 3
last_updated: 2025-12-06
---

# Module 1: ROS 2 Fundamentals

Welcome to Module 1! This module provides the essential foundation for building any robot application. You'll master ROS 2 (Robot Operating System 2), the industry-standard middleware that powers everything from research robots to self-driving cars and humanoid platforms.

## Module Overview

**Duration**: 36 minutes (3 chapters × 12 min each)

**Prerequisites**:
- Completed [Quick Start Guide](../quick-start.md)
- Basic Python programming knowledge
- Ubuntu 22.04 with ROS 2 Humble installed

**What You'll Build**:
- Multiple publisher/subscriber nodes
- Service-based request-response systems
- A complete URDF humanoid robot model

## Learning Objectives

By the end of this module, you will be able to:

1. **Understand ROS 2 Architecture**
   - Explain the node-topic-message communication pattern
   - Differentiate between topics (streaming data) and services (request-response)
   - Navigate the ROS 2 ecosystem (packages, workspaces, build tools)

2. **Create Functional Robot Nodes**
   - Write publisher and subscriber nodes in Python
   - Implement custom message types
   - Handle multiple topics within a single node

3. **Design Robot Models**
   - Create URDF (Unified Robot Description Format) files
   - Define links, joints, and visual properties
   - Model humanoid robot kinematics

4. **Debug and Inspect Systems**
   - Use ROS 2 CLI tools (`ros2 topic`, `ros2 node`, `ros2 service`)
   - Visualize robot models in RViz
   - Monitor message flow with RQT

## Module Structure

### [Chapter 1: ROS 2 Basics & Your First Node](./01-ros2-basics-first-node.md) (12 min)

**Focus**: Core concepts and hands-on node creation

Topics covered:
- ROS 2 graph architecture (nodes, topics, messages)
- Publisher-subscriber pattern
- Creating your first autonomous node
- Message types and interfaces
- Build systems (colcon)

**Hands-on project**: Multi-node communication system

### [Chapter 2: Topics & Services](./02-topics-services.md) (12 min)

**Focus**: Communication patterns for different robot tasks

Topics covered:
- Topics for continuous data streams (sensors, telemetry)
- Services for request-response operations (commands, queries)
- Quality of Service (QoS) settings for reliability
- Custom message and service definitions

**Hands-on project**: Robot control system with both topics and services

### [Chapter 3: URDF for Humanoid Robots](./03-urdf-humanoids.md) (12 min)

**Focus**: Describing robot physical structure

Topics covered:
- URDF file structure (XML-based robot description)
- Links (robot body parts) and joints (connections)
- Coordinate frames and transformations
- Visual and collision geometry
- Humanoid-specific considerations (balance, anthropomorphic proportions)

**Hands-on project**: Build a simplified humanoid robot model

## Why ROS 2?

ROS 2 has become the de facto standard for robot software development because it provides:

✅ **Modularity**: Break complex robot systems into manageable, testable components
✅ **Reusability**: Share code across projects and organizations (thousands of open-source packages)
✅ **Language Flexibility**: Write nodes in Python, C++, or other languages that interoperate seamlessly
✅ **Tool Ecosystem**: Extensive debugging, visualization, and simulation tools
✅ **Industry Adoption**: Used by Boston Dynamics, NASA, Waymo, Tesla, and research labs worldwide

## From ROS to Physical AI

ROS 2 is the foundation layer for Physical AI systems. In later modules, you'll see how:

- **Module 2 (Gazebo)**: ROS 2 nodes control simulated robots with realistic physics
- **Module 3 (Isaac)**: ROS 2 integrates with GPU-accelerated perception pipelines
- **Module 4 (VLA)**: ROS 2 connects voice/LLM planning to robot actions

Understanding ROS 2 is essential for all advanced topics in this book.

## Real-World Applications

Robots using ROS 2 include:

- **Humanoid Platforms**: Tesla Optimus, Figure 01, Agility Robotics Digit
- **Autonomous Vehicles**: Waymo, Cruise (ROS 2 for sensor fusion)
- **Warehouse Robots**: Fetch Robotics, Locus Robotics (mobile manipulation)
- **Research**: NASA Mars rovers, space station robots
- **Education**: TurtleBot, ROSbot (learning platforms)

## Module Completion Checklist

After finishing all three chapters, you should be able to:

- [ ] Create and run multiple ROS 2 nodes simultaneously
- [ ] Explain the difference between topics and services
- [ ] Write custom message and service definitions
- [ ] Build a URDF robot model from scratch
- [ ] Visualize robots in RViz
- [ ] Debug communication issues using ROS 2 CLI tools

## Getting Help

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS Answers**: https://answers.ros.org/
- **Glossary**: See our [Glossary](../glossary.md) for term definitions

## Next Steps

Ready to dive in? Start with **[Chapter 1: ROS 2 Basics & Your First Node](./01-ros2-basics-first-node.md)** to master the fundamentals!

---

**Estimated Time**: Plan for 45-60 minutes to complete all chapters with hands-on exercises. Take breaks as needed!
