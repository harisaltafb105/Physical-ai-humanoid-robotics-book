---
id: urdf-humanoids
title: 'Chapter 3: URDF for Humanoid Robots'
sidebar_position: 3
last_updated: 2025-12-06
---

# Chapter 3: URDF for Humanoid Robots

## Learning Objectives

After completing this chapter, you will be able to:

- Create URDF (Unified Robot Description Format) files
- Define links (body parts) and joints (connections)
- Visualize robots in RViz
- Model humanoid robot kinematic chains

**Reading time**: 12 minutes

## Prerequisites

- Completed [Chapter 2: Topics & Services](./02-topics-services.md)
- Basic XML/HTML understanding

## Introduction

URDF is an XML-based format for describing robot geometry, kinematics, dynamics, and visual properties. It's the standard way to represent robots in ROS 2.

## URDF Building Blocks

### Links - Robot Body Parts

A **link** represents a rigid body component (torso, arm, leg, head).

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>  <!-- width, depth, height in meters -->
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>  <!-- R G B Alpha -->
    </material>
  </visual>
</link>
```

### Joints - Connections Between Links

A **joint** connects two links and defines their motion relationship.

```xml
<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>  <!-- position, rotation -->
  <axis xyz="0 0 1"/>  <!-- rotation axis (Z-axis for yaw) -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>
```

**Joint types**:
- `fixed` - No motion (rigid connection)
- `revolute` - Rotation with limits (like elbow, knee)
- `continuous` - Unlimited rotation (like wheels)
- `prismatic` - Linear motion (like telescoping antenna)

## Hands-On: Build a Simplified Humanoid

Let's create a minimal humanoid robot with torso, head, and two arms.

### Step 1: Create URDF File

Create `simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<!-- Simple humanoid robot description
     Part of Module 1, Chapter 3: URDF for Humanoids
     Tested with: ROS 2 Humble, Ubuntu 22.04 -->

<robot name="simple_humanoid">

  <!-- BASE LINK (required by ROS 2) -->
  <link name="base_link"/>

  <!-- TORSO -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>

  <!-- HEAD -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.8 0.7 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw rotation -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- LEFT ARM -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.15 0.2" rpy="0 1.57 0"/>  <!-- Rotate to point down -->
    <axis xyz="0 0 1"/>  <!-- Shoulder abduction -->
    <limit lower="-3.14" upper="3.14" effort="20" velocity="1.5"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.035"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Elbow flexion -->
    <limit lower="0" upper="2.79" effort="15" velocity="1.5"/>
  </joint>

  <!-- RIGHT ARM (mirrored) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.15 0.2" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="1.5"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.035"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.79" effort="15" velocity="1.5"/>
  </joint>

</robot>
```

### Step 2: Validate URDF

```bash
# Check for syntax errors
check_urdf simple_humanoid.urdf
```

**Expected output**:
```
robot name is: simple_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  torso
        child(1):  head
        child(2):  left_upper_arm
            child(1):  left_forearm
        child(3):  right_upper_arm
            child(1):  right_forearm
```

### Step 3: Visualize in RViz

```bash
# Install joint_state_publisher and robot_state_publisher
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher

# Launch visualization
ros2 launch urdf_tutorial display.launch.py model:=simple_humanoid.urdf
```

This opens RViz showing your humanoid robot! Use the GUI sliders to move joints.

## Humanoid-Specific Considerations

### 1. Center of Mass

For bipedal stability, ensure torso and head don't make the robot top-heavy:

```xml
<inertial>
  <mass value="5.0"/>
  <origin xyz="0 0 0.1"/>  <!-- Lower CoM -->
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

### 2. Anthropomorphic Proportions

Typical humanoid ratios:
- Torso height: ~40% of total height
- Leg length: ~50% of total height
- Arm reach: ~45% of total height

### 3. Joint Limits

Match human range of motion:
- Elbow: 0° to 160° (0 to 2.79 rad)
- Knee: 0° to 140° (0 to 2.44 rad)
- Shoulder: -180° to 180° (-3.14 to 3.14 rad)

## Debugging URDF

### Common Issues

**Issue**: Robot parts are disconnected

**Solution**: Check `<parent>` and `<child>` link names match `<link name="...">`

**Issue**: Robot appears at wrong location

**Solution**: Verify `<origin xyz="..." rpy="..."/>` values

**Issue**: Joints don't move in RViz

**Solution**: Ensure `joint_state_publisher_gui` is running

## Summary

You've learned:

✅ URDF **links** represent rigid body parts
✅ URDF **joints** connect links and define motion
✅ **Joint types**: fixed, revolute, continuous, prismatic
✅ How to create a **humanoid robot** model
✅ Visualizing robots in **RViz**
✅ Humanoid design considerations (CoM, proportions, joint limits)

## Next Steps

Congratulations on completing Module 1! You now understand ROS 2 fundamentals.

Proceed to **[Module 2: Gazebo Simulation](../module-2-gazebo/index.md)** to simulate physics-based robots, or explore **[Module 3: NVIDIA Isaac](../module-3-isaac/index.md)** for photorealistic simulation.

## Further Reading

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [RViz User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
- [Humanoid Robotics Open Platform](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3)
