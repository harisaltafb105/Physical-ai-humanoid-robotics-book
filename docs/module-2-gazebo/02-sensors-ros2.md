---
id: sensors-ros2
title: 'Chapter 2: Sensors & ROS 2 Integration'
sidebar_position: 2
last_updated: 2025-12-06
---

# Chapter 2: Sensors & ROS 2 Integration

## Learning Objectives

After completing this chapter, you will be able to:

- Configure virtual LiDAR, depth cameras, and IMUs in Gazebo
- Publish simulated sensor data to ROS 2 topics
- Subscribe to sensor data from ROS 2 nodes
- Control simulated robots via ROS 2 command topics
- Visualize sensor data in RViz
- Build a complete sim-to-real perception pipeline

**Reading time**: 12 minutes

## Why Simulated Sensors?

### Real Sensors Are Expensive and Fragile

- **Intel RealSense D455**: $329 (and breaks if robot falls!)
- **Velodyne LiDAR VLP-16**: $4,000+
- **Xsens IMU**: $2,000+

In simulation, add unlimited sensors at zero cost:
- 10 cameras on a robot? No problem.
- Test LiDAR failures? Just disable the plugin.
- Compare sensor configurations instantly.

### Sim-to-Real Transfer

Algorithms trained on simulated sensor data transfer surprisingly well to real robots:

✅ **Domain randomization**: Vary lighting, textures, noise → robust perception
✅ **Infinite data**: Generate millions of labeled images without human annotation
✅ **Controlled testing**: Test edge cases (darkness, fog, sensor failures)

**Example**: Google's Everyday Robots project trained manipulation skills 100% in simulation, then deployed to physical robots with minimal tuning.

## Virtual Sensors in Gazebo

### 1. LiDAR (Light Detection and Ranging)

LiDAR emits laser beams and measures time-of-flight to obstacles, producing **point clouds** (3D arrays of x,y,z coordinates).

#### Add LiDAR to Robot (SDF/URDF)

```xml
<sensor name="lidar_sensor" type="gpu_lidar">
  <topic>/scan</topic>
  <update_rate>10</update_rate>  <!-- 10 Hz -->
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>        <!-- Number of rays -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180° -->
        <max_angle>3.14159</max_angle>   <!-- +180° -->
      </horizontal>
      <vertical>
        <samples>1</samples>  <!-- 2D LiDAR (single row) -->
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>   <!-- Minimum detection: 10 cm -->
      <max>30.0</max>  <!-- Maximum range: 30 meters -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1 cm noise -->
    </noise>
  </lidar>
</sensor>
```

#### Read LiDAR Data in ROS 2

```python
#!/usr/bin/env python3
"""
LiDAR subscriber node - reads simulated scan data
Part of Module 2, Chapter 2: Sensors & ROS 2 Integration
Tested with: ROS 2 Humble, Gazebo Garden, Python 3.10
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR topic from Gazebo
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        # msg.ranges: array of distances (in meters)
        # msg.angle_min, msg.angle_max: scan range
        # msg.angle_increment: angular resolution

        min_distance = min(msg.ranges)
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')

        # Collision avoidance logic
        if min_distance < 0.5:
            self.get_logger().warn('⚠️  Obstacle too close! Stop robot.')

def main():
    rclpy.init()
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output (when robot approaches wall):
# [INFO] Closest obstacle: 2.34m
# [INFO] Closest obstacle: 1.87m
# [INFO] Closest obstacle: 0.92m
# [WARN] ⚠️  Obstacle too close! Stop robot.
```

### 2. Depth Camera (RGB-D Sensor)

Depth cameras (e.g., Intel RealSense, Kinect) provide RGB image + depth map.

#### Add Depth Camera (SDF)

```xml
<sensor name="depth_camera" type="depth_camera">
  <topic>/camera/depth</topic>
  <update_rate>30</update_rate>  <!-- 30 FPS -->
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° FOV -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>   <!-- Min depth: 10 cm -->
      <far>10.0</far>    <!-- Max depth: 10 meters -->
    </clip>
    <depth_camera>
      <output>depths</output>
    </depth_camera>
  </camera>
</sensor>
```

#### Process Depth Images in ROS 2

```python
#!/usr/bin/env python3
"""
Depth camera subscriber - finds nearest object in view
Part of Module 2, Chapter 2
Tested with: ROS 2 Humble, Python 3.10, cv_bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthCameraSubscriber(Node):
    def __init__(self):
        super().__init__('depth_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        # Convert ROS Image message to NumPy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Find nearest pixel
        min_depth = np.nanmin(depth_image)  # Ignore NaN (invalid pixels)

        self.get_logger().info(f'Nearest object: {min_depth:.2f}m')

        # Example: Detect if humanoid hand is reaching for object
        if 0.3 < min_depth < 0.6:  # 30-60 cm (arm's reach)
            self.get_logger().info('🤖 Object in grasp range!')

def main():
    rclpy.init()
    node = DepthCameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. IMU (Inertial Measurement Unit)

IMUs measure:
- **Linear acceleration** (m/s²) on x, y, z axes
- **Angular velocity** (rad/s) around x, y, z axes
- **Orientation** (quaternion or Euler angles)

Critical for humanoid balance!

#### Add IMU (SDF)

```xml
<sensor name="imu_sensor" type="imu">
  <topic>/imu</topic>
  <update_rate>100</update_rate>  <!-- 100 Hz (fast for balance control) -->
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><stddev>0.01</stddev></noise></x>
      <y><noise type="gaussian"><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><stddev>0.01</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><stddev>0.1</stddev></noise></x>
      <y><noise type="gaussian"><stddev>0.1</stddev></noise></y>
      <z><noise type="gaussian"><stddev>0.1</stddev></noise></z>
    </linear_acceleration>
  </imu>
</sensor>
```

#### Use IMU for Balance Detection

```python
#!/usr/bin/env python3
"""
IMU subscriber - detect if robot is falling
Part of Module 2, Chapter 2
Tested with: ROS 2 Humble, Python 3.10
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Extract orientation (quaternion)
        quat = msg.orientation

        # Convert to Euler angles (roll, pitch, yaw)
        # Simplified: full conversion needs quaternion math
        roll = math.atan2(2*(quat.w*quat.x + quat.y*quat.z),
                          1 - 2*(quat.x**2 + quat.y**2))

        roll_deg = math.degrees(roll)

        # Humanoid balance threshold: ±15° is safe
        if abs(roll_deg) > 15:
            self.get_logger().error(f'⚠️  Robot tilting! Roll: {roll_deg:.1f}°')
        else:
            self.get_logger().info(f'✅ Balanced. Roll: {roll_deg:.1f}°')

def main():
    rclpy.init()
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output:
# [INFO] ✅ Balanced. Roll: 2.3°
# [INFO] ✅ Balanced. Roll: -1.8°
# [ERROR] ⚠️  Robot tilting! Roll: 18.7°  (robot is falling!)
```

## Gazebo-ROS 2 Bridge

### How Sensors Publish to ROS 2

Gazebo plugins automatically publish sensor data to ROS 2 topics:

```text
┌─────────────────────────┐
│   Gazebo Simulation     │
│  ┌──────────────────┐   │
│  │ LiDAR Plugin     │───┼─── /scan (sensor_msgs/LaserScan)
│  └──────────────────┘   │
│  ┌──────────────────┐   │
│  │ Camera Plugin    │───┼─── /camera/image_raw (sensor_msgs/Image)
│  └──────────────────┘   │
│  ┌──────────────────┐   │
│  │ IMU Plugin       │───┼─── /imu (sensor_msgs/Imu)
│  └──────────────────┘   │
└─────────────────────────┘
         ↓ ROS 2 Topics
┌─────────────────────────┐
│   Your ROS 2 Nodes      │
│  (Perception, Control)  │
└─────────────────────────┘
```

### Controlling Robots from ROS 2

Subscribe to `/cmd_vel` in Gazebo to receive velocity commands:

```python
#!/usr/bin/env python3
"""
Teleoperation publisher - control simulated robot with keyboard
Part of Module 2, Chapter 2
Tested with: ROS 2 Humble, Python 3.10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.publisher.publish(msg)
        self.get_logger().info(f'Sending: linear={self.linear_vel}, angular={self.angular_vel}')

    def set_velocity(self, linear, angular):
        self.linear_vel = linear
        self.angular_vel = angular

def main():
    rclpy.init()
    node = TeleopNode()

    # Example: Move forward for 3 seconds
    node.set_velocity(linear=0.5, angular=0.0)
    rclpy.spin_once(node, timeout_sec=3.0)

    # Stop
    node.set_velocity(linear=0.0, angular=0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visualizing Sensors in RViz

RViz is ROS's 3D visualization tool. View sensor data in real-time:

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch my_robot_description gazebo.launch.py

# Terminal 2: Launch RViz
rviz2

# In RViz:
# 1. Add → LaserScan → Topic: /scan
# 2. Add → Image → Topic: /camera/image_raw
# 3. Add → TF → Show coordinate frames
```

You'll see:
- **LiDAR rays** emanating from robot, hitting obstacles
- **Camera feed** from robot's perspective
- **Coordinate frames** (x=red, y=green, z=blue) for every link

## Hands-On Exercise: Obstacle Avoidance

Combine LiDAR sensing + velocity control:

```python
#!/usr/bin/env python3
"""
Simple obstacle avoidance using LiDAR
Stops robot when obstacle detected within 1 meter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_detected = False

    def scan_callback(self, msg):
        # Check minimum distance in front (±30° cone)
        front_ranges = msg.ranges[len(msg.ranges)//2 - 30:len(msg.ranges)//2 + 30]
        min_dist = min(front_ranges) if front_ranges else float('inf')

        cmd = Twist()
        if min_dist < 1.0:  # Obstacle within 1 meter
            cmd.linear.x = 0.0  # Stop
            self.get_logger().warn(f'🛑 Obstacle at {min_dist:.2f}m - STOP')
        else:
            cmd.linear.x = 0.3  # Move forward at 0.3 m/s
            self.get_logger().info(f'✅ Clear ahead ({min_dist:.2f}m) - Moving')

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Test it**: Place obstacles in Gazebo world, watch robot stop automatically!

## Summary

In this chapter, you learned:

✅ **Virtual sensors**: LiDAR (point clouds), depth cameras (RGB-D), IMUs (orientation)
✅ **Gazebo-ROS 2 bridge**: Sensors publish to topics, robots subscribe to `/cmd_vel`
✅ **Practical skills**: Read sensor data, control robots, visualize in RViz
✅ **Complete loop**: Sense → Process → Act

**Key takeaway**: Simulated sensors are indistinguishable from real ones to your ROS 2 code. Write once, run in simulation and on hardware.

## Next Module

Congratulations on completing Module 2! You now have a functional digital twin of a robot with realistic sensors.

In **[Module 3: NVIDIA Isaac Sim](../module-3-isaac/index.md)**, you'll level up to:
- **Photorealistic rendering** (Isaac Sim vs Gazebo's basic graphics)
- **GPU-accelerated perception** (10x faster than CPU)
- **Synthetic data generation** for training ML models
- **Hardware deployment** on NVIDIA Jetson edge devices

Ready for the cutting edge? Let's go! 🚀

---

**Hands-on challenge**: Add a depth camera to a humanoid robot arm. Use the depth data to detect when the gripper is within 50 cm of an object, then trigger a "grasp" action.
