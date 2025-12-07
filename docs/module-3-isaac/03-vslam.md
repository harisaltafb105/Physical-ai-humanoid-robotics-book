---
id: vslam
title: 'Chapter 3: Visual SLAM with Isaac ROS'
sidebar_position: 3
last_updated: 2025-12-06
---

# Chapter 3: Visual SLAM with Isaac ROS

## Learning Objectives

- Understand Visual SLAM fundamentals and applications
- Configure Isaac ROS cuVSLAM for real-time localization
- Build 3D maps from camera feeds
- Achieve sub-centimeter pose accuracy at 1000 Hz
- Integrate VSLAM with navigation and manipulation

**Reading time**: 12 minutes

## What is VSLAM?

**Visual Simultaneous Localization and Mapping** (VSLAM) answers two questions:
1. **Where am I?** (Localization)
2. **What does the environment look like?** (Mapping)

Using only **cameras**—no LiDAR, GPS, or wheel odometry required.

### Why VSLAM for Humanoid Robots?

✅ **Cheap sensors**: Cameras cost $50 vs. $4,000 for LiDAR
✅ **Rich information**: RGB provides texture, semantics (recognize "door" vs. "wall")
✅ **Indoor navigation**: Works where GPS fails (buildings, warehouses)
✅ **Manipulation**: Depth estimation for grasping objects

**Real-world example**: Boston Dynamics' Atlas humanoid uses stereo cameras for VSLAM to navigate obstacle courses.

## VSLAM Pipeline

```text
┌─────────────────────────────────────────────────────┐
│          Visual SLAM Workflow                       │
└─────────────────────────────────────────────────────┘
        ↓
┌──────────────┐
│ 1. Capture   │ → Camera images (stereo or RGB-D)
│   Images     │
└──────────────┘
        ↓
┌──────────────┐
│ 2. Extract   │ → Feature points (ORB, SIFT, FAST)
│   Features   │
└──────────────┘
        ↓
┌──────────────┐
│ 3. Match     │ → Track features across frames
│   Features   │
└──────────────┘
        ↓
┌──────────────┐
│ 4. Estimate  │ → 3D position (x, y, z, roll, pitch, yaw)
│   Pose       │
└──────────────┘
        ↓
┌──────────────┐
│ 5. Build Map │ → Point cloud or occupancy grid
└──────────────┘
```

### Feature Detection Example

```python
import cv2

# Load image
img = cv2.imread('/tmp/camera_frame.jpg', cv2.IMREAD_GRAYSCALE)

# Extract ORB features (Oriented FAST and Rotated BRIEF)
orb = cv2.ORB_create(nfeatures=500)  # Detect up to 500 keypoints
keypoints, descriptors = orb.detectAndCompute(img, None)

# Draw keypoints
img_with_keypoints = cv2.drawKeypoints(img, keypoints, None,
                                        color=(0, 255, 0),
                                        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv2.imshow("Features", img_with_keypoints)
cv2.waitKey(0)

# Expected: 200-500 green circles on corners, edges, texture-rich areas
```

## Isaac ROS cuVSLAM

**cuVSLAM** = CUDA-accelerated VSLAM (GPU-optimized for Jetson/RTX).

**Performance**:
- **CPU-based VSLAM** (ORB-SLAM3): 10-30 FPS
- **Isaac ROS cuVSLAM**: 60-200 FPS (10x faster!)
- **Pose rate**: Up to 1000 Hz on Jetson Orin

### Installation

```bash
# Install Isaac ROS on Jetson or Ubuntu workstation
sudo apt-get install -y ros-humble-isaac-ros-visual-slam

# Verify installation
ros2 pkg list | grep isaac_ros_visual_slam
# Expected: isaac_ros_visual_slam
```

### Launch cuVSLAM

```bash
# Terminal 1: Start cuVSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 2: Publish stereo camera images
# Option A: From Isaac Sim (synthetic)
ros2 run isaac_sim camera_publisher

# Option B: From real Intel RealSense D435i
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_infra1:=true \
    enable_infra2:=true

# Terminal 3: Visualize in RViz
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam)/share/isaac_ros_visual_slam/rviz/default.rviz
```

### VSLAM Node Configuration

```python
#!/usr/bin/env python3
"""
Configure and run Isaac ROS cuVSLAM
Part of Module 3, Chapter 3
Tested with: ROS 2 Humble, Isaac ROS 2.0, Jetson Orin
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import numpy as np

class VSLAMMonitor(Node):
    def __init__(self):
        super().__init__('vslam_monitor')

        # Subscribe to VSLAM pose estimates
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        # Track position for drift analysis
        self.start_position = None
        self.current_position = None

    def odom_callback(self, msg):
        # Extract position (x, y, z) and orientation (quaternion)
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])

        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(f'Starting position: {self.current_position}')

        # Calculate drift (how far from start)
        drift = np.linalg.norm(self.current_position - self.start_position)

        self.get_logger().info(
            f'Position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] | '
            f'Drift: {drift:.3f}m'
        )

        # Warning if drift exceeds threshold (indicates poor tracking)
        if drift > 0.5:  # 50 cm drift
            self.get_logger().warn('⚠️  High drift detected! Check lighting and features.')

def main():
    rclpy.init()
    node = VSLAMMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output (robot moving forward):
# [INFO] Starting position: [0.000, 0.000, 0.000]
# [INFO] Position: [0.123, 0.001, 0.000] | Drift: 0.123m
# [INFO] Position: [0.456, 0.002, 0.001] | Drift: 0.456m
# [INFO] Position: [1.234, 0.005, 0.002] | Drift: 1.234m
```

## Hands-On: Map a Room with VSLAM

### Step 1: Start VSLAM

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### Step 2: Move Robot (Teleoperation)

```bash
# In another terminal: control robot movement
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

**Controls**:
- `i` = forward
- `j` = turn left
- `l` = turn right
- `k` = stop

### Step 3: Observe Map Building

In RViz, you'll see:
- **Green point cloud**: 3D landmarks detected by VSLAM
- **Red trajectory**: Robot's estimated path
- **Blue camera frustum**: Current camera pose

**Expected behavior**:
- As you move, more green points appear (mapping environment)
- Trajectory follows your keyboard commands
- If you return to start, VSLAM detects **loop closure** and corrects drift

### Step 4: Save Map

```bash
ros2 service call /visual_slam/save_map isaac_ros_visual_slam/srv/SaveMap \
    "{map_url: '/tmp/office_map.db'}"

# Map saved! Can reload later without re-scanning
```

## Loop Closure Detection

**Problem**: Cumulative drift over time (position error accumulates)
**Solution**: When robot revisits a location, recognize it and correct entire trajectory

### How Loop Closure Works

1. Robot explores new area → drift accumulates (10 cm, 20 cm, 50 cm...)
2. Robot returns to starting point
3. VSLAM recognizes familiar features: "I've been here before!"
4. Adjusts entire trajectory to close the loop

**Result**: Error reduced from 50 cm to < 5 cm.

### Visualizing Loop Closures

```python
#!/usr/bin/env python3
"""
Detect and log loop closures
"""

import rclpy
from rclpy.node import Node
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

class LoopClosureDetector(Node):
    def __init__(self):
        super().__init__('loop_closure_detector')
        self.status_sub = self.create_subscription(
            VisualSlamStatus,
            '/visual_slam/status',
            self.status_callback,
            10
        )
        self.loop_closure_count = 0

    def status_callback(self, msg):
        if msg.loop_closure_detected:
            self.loop_closure_count += 1
            self.get_logger().info(
                f'🔄 Loop closure #{self.loop_closure_count} detected! '
                f'Pose corrected.'
            )

def main():
    rclpy.init()
    node = LoopClosureDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Stereo vs. Monocular VSLAM

| Feature | Stereo VSLAM | Monocular VSLAM |
|---------|--------------|------------------|
| **Cameras** | 2 (left + right) | 1 |
| **Depth** | Direct (stereo matching) | Estimated (requires motion) |
| **Scale** | Absolute (meters) | Relative (needs initialization) |
| **Accuracy** | High (±1 cm) | Moderate (±5-10 cm) |
| **Cost** | $200+ (RealSense D435i) | $50 (USB webcam) |
| **Best for** | Humanoid manipulation | Aerial drones |

**Recommendation for humanoids**: Use stereo (Intel RealSense D435i) for better depth accuracy.

## Troubleshooting VSLAM

| Issue | Cause | Solution |
|-------|-------|----------|
| "Lost tracking" | Insufficient features | Add more texture (posters, objects) |
| High drift | Fast motion | Slow down robot movement |
| Map not building | Static scene | Move robot to see parallax |
| Loop closure fails | Lighting changed | Use consistent lighting |

## Summary

In this chapter, you learned:

✅ **VSLAM fundamentals**: Localization + mapping using cameras
✅ **Isaac ROS cuVSLAM**: GPU-accelerated 1000 Hz pose estimation
✅ **Loop closure**: Drift correction when revisiting locations
✅ **Hands-on skills**: Map a room, save/load maps, monitor drift

**Key takeaway**: VSLAM enables indoor navigation without expensive LiDAR or GPS. Perfect for humanoid robots operating in buildings.

## Next Steps

In [Chapter 4: Navigation for Humanoids](./04-navigation-humanoids.md), you'll learn how to:
- Use VSLAM maps for autonomous path planning
- Configure Nav2 for bipedal robot locomotion
- Handle balance constraints during navigation
- Plan footsteps for dynamic walking

---

**Hands-on challenge**: Map your entire lab or apartment using VSLAM. How large can you make the map before drift becomes noticeable? (Hint: Test loop closure by returning to start!)
