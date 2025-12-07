---
id: ros2-basics-first-node
title: 'Chapter 1: ROS 2 Basics & Your First Node'
sidebar_position: 1
last_updated: 2025-12-06
---

# Chapter 1: ROS 2 Basics & Your First Node

## Learning Objectives

After completing this chapter, you will be able to:

- Explain the ROS 2 graph architecture (nodes, topics, messages)
- Create publisher and subscriber nodes in Python
- Build and run a ROS 2 workspace with colcon
- Use ROS 2 CLI tools for debugging

**Reading time**: 12 minutes

## Prerequisites

- Completed [Quick Start Guide](../quick-start.md)
- ROS 2 Humble installed on Ubuntu 22.04
- Basic Python programming knowledge

## Introduction

ROS 2 (Robot Operating System 2) is not an operating system but a **middleware framework** that provides:

- **Communication infrastructure** between robot components
- **Build tools** for compiling and packaging code
- **Standard interfaces** for sensors, actuators, and algorithms
- **Debugging and visualization tools**

Think of ROS 2 as the "nervous system" of a robot, allowing different parts to communicate seamlessly.

## Core Concepts

### 1. Nodes

A **node** is an executable program that performs a specific task. Examples:

- Camera driver node (publishes image data)
- Motor controller node (subscribes to velocity commands)
- Path planner node (publishes navigation waypoints)

Nodes are **independent processes** that can run on different machines.

### 2. Topics

A **topic** is a named channel for asynchronous message passing. Nodes publish to topics, and other nodes subscribe to receive messages.

Example: `/camera/image_raw` topic carries camera frames.

### 3. Messages

**Messages** are data structures sent over topics. ROS 2 provides standard message types like:

- `std_msgs/String` - text data
- `geometry_msgs/Twist` - velocity commands
- `sensor_msgs/Image` - camera images

## Hands-On: Build a Publisher-Subscriber System

### Step 1: Create a Workspace

```bash
# Create workspace structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python my_robot_pkg
cd my_robot_pkg/my_robot_pkg
```

### Step 2: Write a Publisher Node

Create `talker.py`:

```python
#!/usr/bin/env python3
"""
Talker node - publishes robot status messages.
Part of Module 1, Chapter 1: ROS 2 Basics & Your First Node
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(0.5, self.publish_status)  # 2 Hz
        self.count = 0

    def publish_status(self):
        msg = String()
        msg.data = f'Robot operational. Heartbeat: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output:
# [INFO] [talker]: Published: "Robot operational. Heartbeat: 0"
# [INFO] [talker]: Published: "Robot operational. Heartbeat: 1"
```

### Step 3: Write a Subscriber Node

Create `listener.py`:

```python
#!/usr/bin/env python3
"""
Listener node - subscribes to robot status.
Part of Module 1, Chapter 1: ROS 2 Basics & Your First Node
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output:
# [INFO] [listener]: Received: "Robot operational. Heartbeat: 5"
```

### Step 4: Update Package Configuration

Edit `setup.py` to add entry points:

```python
entry_points={
    'console_scripts': [
        'talker = my_robot_pkg.talker:main',
        'listener = my_robot_pkg.listener:main',
    ],
},
```

### Step 5: Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build with colcon
colcon build --packages-select my_robot_pkg

# Source the workspace
source install/setup.bash
```

**Expected output**:
```
Starting >>> my_robot_pkg
Finished <<< my_robot_pkg [1.23s]
Summary: 1 package finished [1.45s]
```

### Step 6: Run the Nodes

**Terminal 1** (Talker):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_pkg talker
```

**Terminal 2** (Listener):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_pkg listener
```

## Debugging with ROS 2 CLI Tools

### Inspect Active Nodes

```bash
ros2 node list
```

**Output**:
```
/listener
/talker
```

### Get Node Information

```bash
ros2 node info /talker
```

**Output**:
```
/talker
  Subscribers:

  Publishers:
    /robot_status: std_msgs/msg/String
```

### Monitor Topic Data

```bash
ros2 topic echo /robot_status
```

### Check Publishing Rate

```bash
ros2 topic hz /robot_status
```

**Output**:
```
average rate: 2.000
```

## Summary

You've learned:

✅ ROS 2 **nodes** are independent processes performing specific tasks
✅ **Topics** enable publish-subscribe communication between nodes
✅ **Messages** are typed data structures (like `std_msgs/String`)
✅ **colcon** builds ROS 2 packages
✅ ROS 2 CLI tools (`ros2 node`, `ros2 topic`) help debug systems

## Next Steps

Proceed to **[Chapter 2: Topics & Services](./02-topics-services.md)** to learn request-response communication patterns.

## Further Reading

- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [Standard Message Types](https://github.com/ros2/common_interfaces)
