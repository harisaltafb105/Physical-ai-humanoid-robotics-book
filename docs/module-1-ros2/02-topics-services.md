---
id: topics-services
title: 'Chapter 2: Topics & Services'
sidebar_position: 2
last_updated: 2025-12-06
---

# Chapter 2: Topics & Services

## Learning Objectives

After completing this chapter, you will be able to:

- Distinguish between topics (continuous streams) and services (request-response)
- Implement service servers and clients
- Configure Quality of Service (QoS) settings
- Choose the appropriate communication pattern for robot tasks

**Reading time**: 12 minutes

## Prerequisites

- Completed [Chapter 1: ROS 2 Basics & Your First Node](./01-ros2-basics-first-node.md)
- Understanding of publisher-subscriber pattern

## Topics vs Services: When to Use Which?

### Topics - Continuous Data Streams

**Use for**:
- Sensor data (camera frames, LiDAR scans at 10-30 Hz)
- Telemetry (robot position, battery level)
- Continuous control commands (velocity setpoints)

**Characteristics**:
- One-to-many or many-to-many communication
- Fire-and-forget (no response confirmation)
- Latest data is often most important

### Services - Request-Response

**Use for**:
- Commands that need confirmation (gripper open/close)
- Queries (get current robot state)
- One-time operations (save map, calibrate sensor)

**Characteristics**:
- One-to-one communication
- Blocking call with guaranteed response
- Used for infrequent operations

## Hands-On: Robot Control System

Let's build a robot controller using both topics and services.

### Scenario

- **Topic**: Continuous velocity commands → robot base
- **Service**: Emergency stop command → immediate halt

### Step 1: Create Service Definition

In your package, create `srv/EmergencyStop.srv`:

```
# Request
string reason
---
# Response
bool success
string message
```

Update `CMakeLists.txt` or `package.xml` to include service generation (details in ROS 2 docs).

### Step 2: Service Server (Robot Controller)

Create `robot_controller.py`:

```python
#!/usr/bin/env python3
"""
Robot controller with topic subscriptions and service server.
Part of Module 1, Chapter 2: Topics & Services
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger  # Using standard service for simplicity

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Subscribe to velocity commands
        self.vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        # Create emergency stop service
        self.estop_service = self.create_service(
            Trigger,
            'emergency_stop',
            self.estop_callback
        )

        self.is_stopped = False
        self.get_logger().info('Robot controller ready')

    def velocity_callback(self, msg):
        if not self.is_stopped:
            # In real robot, this would send commands to motors
            self.get_logger().info(
                f'Moving: linear={msg.linear.x:.2f}, '
                f'angular={msg.angular.z:.2f}'
            )
        else:
            self.get_logger().warn('Robot is stopped! Ignoring velocity command.')

    def estop_callback(self, request, response):
        self.is_stopped = True
        response.success = True
        response.message = 'Emergency stop activated!'
        self.get_logger().warn('EMERGENCY STOP TRIGGERED')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output:
# [INFO] [robot_controller]: Robot controller ready
# [INFO] [robot_controller]: Moving: linear=0.50, angular=0.00
# [WARN] [robot_controller]: EMERGENCY STOP TRIGGERED
```

### Step 3: Velocity Publisher

Create `velocity_publisher.py`:

```python
#!/usr/bin/env python3
"""
Publishes velocity commands to robot.
Part of Module 1, Chapter 2: Topics & Services
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.1  # Turn slightly
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published velocity command')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Service Client (Emergency Stop Button)

Create `estop_client.py`:

```python
#!/usr/bin/env python3
"""
Emergency stop service client.
Part of Module 1, Chapter 2: Topics & Services
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class EmergencyStopClient(Node):
    def __init__(self):
        super().__init__('estop_client')
        self.client = self.create_client(Trigger, 'emergency_stop')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for emergency_stop service...')

    def trigger_estop(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            self.get_logger().info(f'E-Stop response: {response.message}')
        else:
            self.get_logger().error('E-Stop failed!')

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopClient()
    node.trigger_estop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output:
# [INFO] [estop_client]: E-Stop response: Emergency stop activated!
```

### Step 5: Test the System

**Terminal 1** (Robot Controller):
```bash
ros2 run my_robot_pkg robot_controller
```

**Terminal 2** (Velocity Publisher):
```bash
ros2 run my_robot_pkg velocity_publisher
```

**Terminal 3** (Emergency Stop):
```bash
ros2 run my_robot_pkg estop_client
```

## Quality of Service (QoS)

QoS settings control message delivery reliability and performance.

### Common QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Sensor data - best effort, drop old data
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Control commands - reliable delivery
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Use in publisher/subscriber
self.create_publisher(Twist, 'cmd_vel', qos_profile=control_qos)
```

### When to Use Which QoS

- **BEST_EFFORT**: Sensors (LiDAR, cameras) where latest data matters most
- **RELIABLE**: Commands, state updates where guaranteed delivery is critical

## Debugging Services

### List Available Services

```bash
ros2 service list
```

**Output**:
```
/emergency_stop
/robot_controller/describe_parameters
...
```

### Get Service Type

```bash
ros2 service type /emergency_stop
```

**Output**:
```
std_srvs/srv/Trigger
```

### Call Service from CLI

```bash
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

## Summary

You've learned:

✅ **Topics** for continuous data streams (sensors, commands)
✅ **Services** for request-response operations (queries, one-time commands)
✅ **QoS settings** control reliability and performance
✅ Service **clients** and **servers** pattern
✅ How to combine topics and services in a robot system

## Next Steps

Proceed to **[Chapter 3: URDF for Humanoids](./03-urdf-humanoids.md)** to learn robot description formats.

## Further Reading

- [About QoS Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
