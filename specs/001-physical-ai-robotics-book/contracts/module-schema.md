# Module Schema Contract

**Purpose**: Define the structure and frontmatter requirements for module index files.

## File Location

```
docs/module-<N>-<name>/index.md
```

Where:
- `<N>` = module number (1, 2, 3, 4)
- `<name>` = URL-friendly module identifier (e.g., "ros2", "digital-twin", "nvidia-isaac", "vla")

## YAML Frontmatter Schema

```yaml
---
id: <module_id>              # Required: Unique identifier (e.g., "module-1-ros2")
title: <title_string>        # Required: Display title (e.g., "Module 1: The Robotic Nervous System (ROS 2)")
sidebar_position: <integer>  # Required: Order in sidebar (1-4)
---
```

### Field Specifications

#### `id` (required, string)
- **Format**: `module-<N>-<name>` where N is 1-4
- **Pattern**: `/^module-[1-4]-[a-z0-9-]+$/`
- **Examples**: "module-1-ros2", "module-2-digital-twin"
- **Purpose**: Unique identifier for linking and references

#### `title` (required, string)
- **Format**: "Module <N>: <Full Title>"
- **Length**: 30-80 characters
- **Examples**:
  - "Module 1: The Robotic Nervous System (ROS 2)"
  - "Module 3: The AI-Robot Brain (NVIDIA Isaac)"
- **Purpose**: Display name in sidebar and page header

#### `sidebar_position` (required, integer)
- **Range**: 1-4 (must match module number)
- **Purpose**: Determines order in sidebar navigation
- **Note**: Must be unique across all modules

## Markdown Body Structure

### Required Sections

1. **Module Overview** (immediately after frontmatter)
   - 2-3 paragraphs explaining module purpose
   - What technology this module covers
   - Why it matters in Physical AI context

2. **Learning Objectives** (`## Learning Objectives`)
   - Bulleted list of 3-5 measurable outcomes
   - Start with action verbs (Understand, Create, Implement, etc.)
   - Specific and testable

3. **Prerequisites** (`## Prerequisites`)
   - List of required prior knowledge
   - Reference to previous modules (if applicable)
   - External prerequisites (software, hardware)

4. **Chapter Overview** (`## Chapters in This Module`)
   - Brief description of each chapter (1 sentence each)
   - Listed in order (matches sidebar_position)

5. **What You'll Build** (`## What You'll Build`)
   - Description of hands-on project/tutorial outcomes
   - Concrete examples of what students will create

### Optional Sections

- **Estimated Time**: Total reading time for module (e.g., "~60 minutes")
- **Key Technologies**: List of tools/frameworks covered
- **Next Module Preview**: Teaser for following module

## Example Module Index

```markdown
---
id: module-1-ros2
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_position: 1
---

# Module 1: The Robotic Nervous System (ROS 2)

The Robot Operating System 2 (ROS 2) is the middleware that powers modern robotics. Just as the nervous system coordinates signals between the brain and body, ROS 2 coordinates communication between sensors, algorithms, and actuators in a robot. This module introduces you to ROS 2's core concepts and shows you how to build your first intelligent robot controllers.

By the end of this module, you'll understand how to create distributed robot systems where Python AI agents seamlessly control physical hardware through standardized interfaces.

## Learning Objectives

- Understand ROS 2 as middleware for distributed robot control systems
- Create and execute ROS 2 nodes in Python using the rclpy library
- Implement publisher-subscriber communication via topics for streaming data
- Implement request-response communication via services for one-off queries
- Define robot geometry and kinematics using URDF for humanoid structures

## Prerequisites

- **Programming**: Basic Python (functions, classes, modules)
- **Environment**: Ubuntu 22.04 (native or via Docker) with ROS 2 Humble installed
- **Prior Modules**: None (this is the foundation module)

## Chapters in This Module

1. **ROS 2 Fundamentals**: Learn what ROS 2 is, why it matters, and the core distributed systems concepts
2. **Your First ROS 2 Node**: Hands-on tutorial to create, build, and run a simple publisher node
3. **Topics and Services**: Understand communication patterns and when to use each
4. **Python and rclpy**: Bridge Python AI agents to ROS robot controllers
5. **URDF for Humanoids**: Define robot geometry, joints, and coordinate frames

## What You'll Build

- A "Hello ROS 2" publisher node that sends messages every second
- A subscriber node that receives and processes those messages
- A service that responds to position queries
- A URDF file defining a simple humanoid robot structure
- Integration between a Python AI planner and ROS 2 motor controllers

**Estimated Time**: ~60 minutes total (12 minutes per chapter)

**Next Module**: After mastering ROS 2, you'll move to Module 2 where you'll simulate your robots in Gazebo and Unity before deploying to physical hardware.
```

## Validation Rules

### Pre-Commit Checks

1. **Frontmatter Validation**:
   - All required fields present (id, title, sidebar_position)
   - `id` matches filename pattern (`module-<N>-<name>`)
   - `sidebar_position` matches module number
   - `title` starts with "Module <N>:"

2. **Content Validation**:
   - Contains "## Learning Objectives" section with 3-5 bullet points
   - Contains "## Prerequisites" section
   - Contains "## Chapters in This Module" section
   - Learning objectives use action verbs (Understand, Create, Implement, etc.)

3. **Markdown Linting**:
   - No broken internal links
   - Proper heading hierarchy (no skipped levels)
   - Consistent list formatting

### Build-Time Checks

1. **Chapter Count**: Directory must contain 5-6 chapter files (matches chapter overview)
2. **Sidebar Order**: `sidebar_position` must be unique across all modules
3. **Reading Time**: Sum of chapter reading times should match estimated module time (±10 min)

## Implementation Notes

- **File Creation**: Use contract template when generating module index files
- **Content-First**: Write learning objectives before chapter content
- **Progressive Structure**: Each module should build on previous modules
- **Consistency**: Use same heading structure across all module index files
