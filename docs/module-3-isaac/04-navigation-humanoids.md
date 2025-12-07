---
id: navigation-humanoids
title: 'Chapter 4: Navigation for Bipedal Humanoid Robots'
sidebar_position: 4
last_updated: 2025-12-06
---

# Chapter 4: Navigation for Bipedal Humanoid Robots

## Learning Objectives

- Configure Nav2 stack for humanoid robot navigation
- Understand bipedal locomotion constraints vs. wheeled robots
- Implement footstep planning for dynamic walking
- Handle balance and stability during navigation
- Integrate VSLAM localization with path planning

**Reading time**: 12 minutes

## Why Humanoid Navigation is Hard

### Wheeled Robots vs. Bipedal Humanoids

| Challenge | Wheeled Robot | Humanoid Robot |
|-----------|---------------|----------------|
| **Stability** | Always stable (3-4 wheels) | Must actively balance on 2 feet |
| **Turning** | In-place rotation | Requires footsteps |
| **Obstacles** | Drive around | Can step over |
| **Speed** | Up to 5 m/s | 0.5-1.5 m/s (walking) |
| **Energy** | Efficient | High (lifting legs) |
| **Control complexity** | Low (2 motors) | High (12+ joint DOF per leg) |

**Key insight**: Humanoid navigation requires **footstep planning** + **balance control**, not just path following.

## Nav2 Stack Overview

**Nav2** (Navigation2) is ROS 2's autonomous navigation framework. Components:

```text
┌─────────────────────────────────────────────────┐
│              Nav2 Architecture                  │
└─────────────────────────────────────────────────┘
         ↓
┌──────────────────┐
│ 1. Costmap       │ → Occupancy grid (obstacles = high cost)
└──────────────────┘
         ↓
┌──────────────────┐
│ 2. Global Planner│ → A* path from start → goal
└──────────────────┘
         ↓
┌──────────────────┐
│ 3. Local Planner │ → Dynamic obstacle avoidance
│   (DWB/TEB)      │
└──────────────────┘
         ↓
┌──────────────────┐
│ 4. Controller    │ → Velocity commands (linear, angular)
└──────────────────┘
         ↓
┌──────────────────┐
│ 5. Robot         │ → Execute motion (for humanoids: footsteps!)
└──────────────────┘
```

### Install Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Configuring Nav2 for Humanoids

### Key Differences from Wheeled Robots

1. **Footprint**: Smaller (humanoid = 0.3m wide vs. robot = 0.5m)
2. **Max velocity**: Slower (1.0 m/s walk vs. 2.0 m/s drive)
3. **Rotation**: Cannot spin in place (must step)
4. **Step height**: Can climb stairs (wheeled cannot)

### Nav2 Parameters (humanoid_nav2_params.yaml)

```yaml
# Humanoid-specific navigation parameters
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry  # From cuVSLAM
    transform_tolerance: 0.1

controller_server:
  ros__parameters:
    controller_frequency: 20.0  # 20 Hz (slower than wheeled robots)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 1.0        # 1.0 m/s max walk speed
      min_vel_y: 0.0        # Humanoids don't strafe (holonomic = false)
      max_vel_y: 0.0
      max_vel_theta: 0.5    # 0.5 rad/s max turn rate
      min_speed_xy: 0.1     # Minimum speed to maintain balance
      acc_lim_x: 0.5        # Slow acceleration (avoid tipping)
      acc_lim_theta: 0.5

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.2          # 20 cm goal tolerance (bipedal less precise)
      use_astar: true

costmap_2d:
  global_costmap:
    footprint: [[0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]  # 30cm square
    inflation_radius: 0.3   # Stay 30cm away from obstacles (safety margin)

  local_costmap:
    width: 3.0              # 3m local planning window
    height: 3.0
    resolution: 0.05        # 5cm grid cells
```

## Footstep Planning

### Why Footsteps Matter

Wheeled robots: "Move to (x, y, θ)" → done
Humanoids: "Move to (x, y, θ)" → must plan:
1. Which foot to step first (left or right)?
2. Where to place each footstep?
3. How to shift weight without falling?

### Footstep Planner Node

```python
#!/usr/bin/env python3
"""
Simple footstep planner for humanoid navigation
Part of Module 3, Chapter 4
Tested with: ROS 2 Humble, Python 3.10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import numpy as np

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        # Subscribe to Nav2 path
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        # Publish footstep commands
        self.footstep_pub = self.create_publisher(
            PoseStamped,
            '/footstep_goal',
            10
        )

        self.current_foot = "left"  # Alternate left/right
        self.step_length = 0.3      # 30 cm per step
        self.step_width = 0.2       # 20 cm lateral spacing

    def path_callback(self, msg):
        """Convert Nav2 path to footsteps"""

        poses = msg.poses
        if len(poses) < 2:
            return

        # Calculate total distance
        total_dist = 0.0
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            dist = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_dist += dist

        # Number of steps needed
        num_steps = int(np.ceil(total_dist / self.step_length))

        self.get_logger().info(
            f'Path distance: {total_dist:.2f}m → {num_steps} footsteps'
        )

        # Generate footsteps along path
        for i in range(num_steps):
            # Interpolate along path
            progress = (i / num_steps)
            pose_idx = int(progress * (len(poses) - 1))
            target_pose = poses[pose_idx].pose

            # Offset for left/right foot
            offset = self.step_width / 2 if self.current_foot == "left" else -self.step_width / 2

            footstep = PoseStamped()
            footstep.header.frame_id = "map"
            footstep.header.stamp = self.get_clock().now().to_msg()
            footstep.pose.position.x = target_pose.position.x
            footstep.pose.position.y = target_pose.position.y + offset
            footstep.pose.position.z = 0.0
            footstep.pose.orientation = target_pose.orientation

            self.footstep_pub.publish(footstep)
            self.get_logger().info(
                f'Step {i+1}/{num_steps}: {self.current_foot} foot → '
                f'({footstep.pose.position.x:.2f}, {footstep.pose.position.y:.2f})'
            )

            # Alternate feet
            self.current_foot = "right" if self.current_foot == "left" else "left"

def main():
    rclpy.init()
    node = FootstepPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output (3-meter path):
# [INFO] Path distance: 3.00m → 10 footsteps
# [INFO] Step 1/10: left foot → (0.30, 0.10)
# [INFO] Step 2/10: right foot → (0.60, -0.10)
# [INFO] Step 3/10: left foot → (0.90, 0.10)
# ...
```

## Balance and Stability

### Center of Mass (CoM) Control

**Critical constraint**: CoM must stay within support polygon (footprint).

```python
def check_balance(com_position, left_foot_pos, right_foot_pos):
    """
    Verify CoM is within support polygon (avoids tipping)
    """
    # Support polygon = convex hull of feet in contact
    # For static balance: CoM must be inside polygon

    # Simplified: assume CoM should be between feet
    min_x = min(left_foot_pos[0], right_foot_pos[0])
    max_x = max(left_foot_pos[0], right_foot_pos[0])
    min_y = min(left_foot_pos[1], right_foot_pos[1])
    max_y = max(left_foot_pos[1], right_foot_pos[1])

    if (min_x <= com_position[0] <= max_x and
        min_y <= com_position[1] <= max_y):
        return True  # Balanced
    else:
        return False  # Will tip over!

# Example
left_foot = (0.0, 0.1)   # Left foot at y=+10cm
right_foot = (0.0, -0.1) # Right foot at y=-10cm
com = (0.0, 0.0)         # CoM centered

print(check_balance(com, left_foot, right_foot))  # True (stable)

com_off = (0.0, 0.5)     # CoM shifted 50cm to left
print(check_balance(com_off, left_foot, right_foot))  # False (will fall!)
```

### Zero Moment Point (ZMP)

**ZMP** = point where net moment from gravity and inertia is zero.

**Rule**: If ZMP is inside support polygon → stable. If outside → tipping.

## Hands-On: Autonomous Humanoid Navigation

### Step 1: Launch Nav2

```bash
# Terminal 1: Start Isaac Sim with humanoid
ros2 launch isaac_sim humanoid_sim.launch.py

# Terminal 2: Start cuVSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Launch Nav2
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/path/to/humanoid_nav2_params.yaml
```

### Step 2: Set Navigation Goal

```bash
# Option A: Via command line
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "map"},
      pose: {position: {x: 3.0, y: 2.0, z: 0.0},
             orientation: {w: 1.0}}}'

# Option B: Via RViz
# Click "2D Nav Goal" button, click target location on map
```

**Expected behavior**:
1. Nav2 computes path (green line in RViz)
2. Footstep planner converts path → footsteps
3. Humanoid walks along path, avoiding obstacles
4. Arrives at goal (within 20 cm tolerance)

## Summary

In this chapter, you learned:

✅ **Nav2 for humanoids**: Slower speeds, footstep planning, balance constraints
✅ **Footstep planning**: Convert waypoint paths to left/right foot placements
✅ **Balance control**: CoM and ZMP must stay within support polygon
✅ **Integration**: VSLAM localization + Nav2 planning + footstep execution

**Key takeaway**: Humanoid navigation is fundamentally different from wheeled robots. Success requires integrating perception (VSLAM), planning (Nav2), and control (balance).

## Next Module

Congratulations on completing Module 3! You now have cutting-edge skills in:
- Isaac Sim photorealistic simulation
- Synthetic data generation for ML
- GPU-accelerated perception on Jetson
- Visual SLAM for indoor localization
- Humanoid navigation and footstep planning

In **[Module 4: Vision-Language-Action (VLA)](../module-4-vla/index.md)**, you'll learn the future of robotics:
- **Voice control**: "Pick up the red box" → robot executes
- **LLM planning**: GPT-4 generates robot action sequences
- **Multimodal AI**: Combine vision, language, and action

Ready for the cutting edge of AI robotics? Let's go! 🚀

---

**Hands-on challenge**: Set up a simulated obstacle course in Isaac Sim. Use Nav2 to navigate a humanoid from start to finish, avoiding boxes and navigating around furniture. How fast can you complete the course while maintaining stability?
