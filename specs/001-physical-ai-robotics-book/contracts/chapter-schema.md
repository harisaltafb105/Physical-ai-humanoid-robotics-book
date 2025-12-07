# Chapter Schema Contract

**Purpose**: Define the structure and frontmatter requirements for chapter markdown files.

## File Location

```
docs/module-<N>-<name>/<NN>-<chapter-name>.md
```

Where:
- `<N>` = module number (1, 2, 3, 4)
- `<name>` = module identifier (e.g., "ros2", "digital-twin")
- `<NN>` = chapter number within module (01, 02, 03, etc.)
- `<chapter-name>` = URL-friendly chapter identifier

## YAML Frontmatter Schema

```yaml
---
id: <chapter_id>              # Required: Unique identifier
title: <title_string>         # Required: Display title
sidebar_position: <integer>   # Required: Order within module (1-6)
last_updated: <YYYY-MM-DD>    # Required: ISO 8601 date
---
```

### Field Specifications

#### `id` (required, string)
- **Format**: Kebab-case identifier (e.g., "ros2-first-node", "isaac-sim-setup")
- **Pattern**: `/^[a-z0-9-]+$/`
- **Length**: 10-50 characters
- **Examples**: "ros2-fundamentals", "gazebo-physics", "vla-integration"
- **Purpose**: Unique identifier for cross-references and permalinks
- **Note**: Must be unique across entire documentation site

#### `title` (required, string)
- **Length**: 10-60 characters
- **Format**: Title case without module prefix
- **Examples**:
  - "Your First ROS 2 Node"
  - "Physics Simulation in Gazebo"
  - "Integrating VLA with ROS 2 Robots"
- **Purpose**: Display name in sidebar, page header, and breadcrumbs

#### `sidebar_position` (required, integer)
- **Range**: 1-6 (must be unique within module)
- **Purpose**: Determines chapter order in sidebar
- **Note**: Should match `<NN>` prefix in filename (e.g., "02-first-node.md" → sidebar_position: 2)

#### `last_updated` (required, string)
- **Format**: ISO 8601 date (YYYY-MM-DD)
- **Example**: "2025-12-06"
- **Purpose**: Track content freshness, displayed to readers
- **Note**: Update whenever chapter content is modified

## Markdown Body Structure

### Required Sections (in order)

1. **Learning Objectives** (`## Learning Objectives`)
   - Bulleted list of 3-5 measurable outcomes
   - Start with action verbs (Create, Understand, Implement, Configure, etc.)
   - Specific to this chapter (not entire module)

2. **Prerequisites** (`## Prerequisites`)
   - List of required prior knowledge
   - Reference to previous chapters (with markdown links)
   - Software/hardware requirements (if applicable)

3. **Introduction** (`## Introduction`)
   - 2-3 paragraphs (200-300 words)
   - **Why**: Why this topic matters
   - **What**: What you'll learn in this chapter
   - **Preview**: Brief overview of hands-on component (if applicable)

4. **Core Concepts** (`## Core Concepts`)
   - 2-4 subsections (### headings)
   - Each subsection covers one concept
   - Include diagrams or code examples
   - Build from simple to complex

5. **Hands-On Tutorial** (`## Hands-On Tutorial`) *[if applicable]*
   - Step-by-step walkthrough
   - Numbered steps (clear progression)
   - Code blocks with inline comments
   - Expected output after each key step

6. **Summary** (`## Summary`)
   - Bulleted list of key takeaways (3-5 points)
   - What you learned
   - How it connects to module objectives

7. **Next Steps** (`## Next Steps`)
   - 1-2 sentence preview of next chapter
   - Optional: Links to additional resources (external documentation, papers)

### Optional Sections

- **Common Pitfalls** (`## Common Pitfalls`): Troubleshooting common errors
- **Advanced Topics** (`## Advanced Topics`): Optional deep dives (collapsed by default)
- **Further Reading** (`## Further Reading`): External links to documentation, papers, tutorials

## Content Constraints

### Reading Time Budget

- **Target**: 12 minutes per chapter
- **Word Count**: 1,500-2,000 words
- **Calculation**: ~150 words/minute average reading speed
- **Includes**: Time to read code examples (slower than prose)

### Code Example Requirements

- **Minimum**: 1 runnable code example per chapter
- **Format**: Fenced code blocks with language annotation
- **Header Comment**: Description, module/chapter reference, tested environment
- **Inline Comments**: Explain non-obvious logic
- **Expected Output**: Show what user should see when running code
- **Runnable**: Copy-paste must work without modifications

### Visual Requirements

- **Minimum**: 1 visual per chapter (diagram OR screenshot)
- **Format**: SVG for diagrams, PNG for screenshots
- **Alt Text**: Descriptive accessibility text
- **Caption**: Figure number and explanation (e.g., "**Figure 1.2**: ROS 2 node communication pattern")
- **Reference**: Mentioned in body text ("see Figure 1.2")

## Example Chapter

```markdown
---
id: ros2-first-node
title: Your First ROS 2 Node
sidebar_position: 2
last_updated: 2025-12-06
---

# Your First ROS 2 Node

## Learning Objectives

- Create a ROS 2 Python node using the rclpy library
- Understand the publisher pattern for sending messages
- Run and debug a simple ROS 2 node
- Read node output using ROS 2 command-line tools

## Prerequisites

- Completed [ROS 2 Fundamentals](./01-ros2-fundamentals.md)
- ROS 2 Humble installed (see [Quick Start Guide](/quick-start))
- Basic Python programming (functions, classes)
- Text editor (VS Code, vim, or nano)

## Introduction

Creating your first ROS 2 node is the "Hello World" of robotics programming. In this chapter, you'll build a simple publisher node that sends messages every second. This hands-on tutorial will teach you the fundamental pattern used by thousands of real-world robot applications.

By the end of this chapter, you'll have a working ROS 2 node running on your machine. More importantly, you'll understand the core structure that every ROS 2 node follows - a pattern you'll use throughout your robotics career.

Let's dive in and get your first node running in the next 12 minutes.

## Core Concepts

### The Publisher Pattern

ROS 2 uses the **publisher-subscriber pattern** for streaming data. A publisher node sends messages to a topic, and any number of subscriber nodes can receive those messages. Think of it like a radio station (publisher) broadcasting to listeners (subscribers).

![ROS 2 Publisher-Subscriber Pattern](../../static/img/ros2-node-diagram.svg)

**Figure 2.1**: ROS 2 Publisher-Subscriber Pattern. The publisher sends messages to a topic; subscribers receive them asynchronously.

### Node Lifecycle

Every ROS 2 node goes through these stages:

1. **Initialization**: Set up the node with a unique name
2. **Configuration**: Create publishers, subscribers, timers
3. **Execution**: Spin (process callbacks and events)
4. **Shutdown**: Clean up resources before exit

### The rclpy Library

`rclpy` (ROS Client Library for Python) is the official Python interface to ROS 2. It provides classes for nodes, publishers, subscribers, and more. Every Python ROS 2 program imports `rclpy`.

## Hands-On Tutorial

Let's build a simple publisher node step-by-step.

### Step 1: Create the Python File

Create a new file called `hello_publisher.py`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
touch hello_publisher.py
chmod +x hello_publisher.py
```

### Step 2: Write the Node Code

Open `hello_publisher.py` and add this code:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 publisher node that sends string messages.
Part of Module 1, Chapter 2: Your First ROS 2 Node
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')  # Node name
        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        # Timer: publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Hello Publisher node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    node = HelloPublisher()
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Code Walkthrough**:
- **Line 9-10**: Import ROS 2 libraries and message type
- **Line 12-23**: Define node class (inherits from `Node`)
- **Line 14**: Initialize with node name `'hello_publisher'`
- **Line 16**: Create publisher for topic `'hello_topic'`
- **Line 18**: Set up timer to call `timer_callback()` every 1 second
- **Line 21-26**: Callback function that publishes messages
- **Line 28-37**: Main function that initializes and runs the node

### Step 3: Run the Node

Execute the node:

```bash
python3 hello_publisher.py
```

**Expected Output**:
```
[INFO] [1733494800.123456789] [hello_publisher]: Hello Publisher node started
[INFO] [1733494801.123456789] [hello_publisher]: Publishing: "Hello ROS 2! Count: 0"
[INFO] [1733494802.123456789] [hello_publisher]: Publishing: "Hello ROS 2! Count: 1"
[INFO] [1733494803.123456789] [hello_publisher]: Publishing: "Hello ROS 2! Count: 2"
...
```

Press `Ctrl+C` to stop the node.

### Step 4: Inspect with ROS 2 Tools

In a new terminal, check that the topic exists:

```bash
ros2 topic list
```

You should see `/hello_topic` in the list. Read messages from the topic:

```bash
ros2 topic echo /hello_topic
```

You'll see the messages your node is publishing!

## Summary

- **Created** a ROS 2 Python node using `rclpy`
- **Implemented** the publisher pattern to send messages to a topic
- **Configured** a timer to publish messages every second
- **Ran** the node and verified output using ROS 2 command-line tools
- **Learned** the core node structure (init, configure, spin, shutdown)

This pattern - create a node, configure publishers/subscribers, and spin - is the foundation of every ROS 2 program you'll write.

## Next Steps

In [Chapter 3: Topics and Services](./03-topics-and-services.md), you'll learn when to use the publisher-subscriber pattern (topics) versus the request-response pattern (services). You'll also build a subscriber to receive the messages your publisher is sending.

**Further Reading**:
- [Official rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Publisher-Subscriber Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
```

## Validation Rules

### Pre-Commit Checks

1. **Frontmatter Validation**:
   - All required fields present (id, title, sidebar_position, last_updated)
   - `id` is unique across entire site
   - `sidebar_position` is unique within module
   - `last_updated` is valid ISO 8601 date

2. **Content Structure Validation**:
   - Contains all required sections in order
   - Learning Objectives has 3-5 bullet points
   - Summary has 3-5 bullet points
   - At least 1 code example OR 1 visual

3. **Code Example Validation**:
   - Header comment includes description, module/chapter, tested environment
   - Language annotation present (```python, ```bash, etc.)
   - Expected output included (in comments or following text)

4. **Markdown Linting**:
   - No broken internal links
   - Proper heading hierarchy (no skipped levels: ## → #### invalid)
   - Consistent list formatting (bullets vs. numbers)
   - No trailing whitespace

### Build-Time Checks

1. **Reading Time**: Word count between 1,500-2,000 words (10-15 min reading time)
2. **Code Runnability**: Code examples should be testable (future enhancement: automated testing)
3. **Image Existence**: Referenced images must exist in `static/img/` directory

## Implementation Notes

- **Template Usage**: Use this contract as template when creating new chapters
- **Consistency**: Maintain same section order across all chapters for reader familiarity
- **Conciseness**: Strict 12-minute budget - cut ruthlessly to stay within limits
- **Hands-On Focus**: Chapters with tutorials are more engaging than theory-only chapters
