# Code Example Schema Contract

**Purpose**: Define formatting standards for runnable code snippets embedded in chapters.

## Overview

All code examples in the book must be:
- **Runnable**: Copy-paste into terminal or file must work without modification
- **Commented**: Inline comments explain non-obvious logic
- **Versioned**: Specify tested environment (ROS 2 version, Python version, etc.)
- **Demonstrated**: Show expected output

## Code Block Format

### Markdown Syntax

````markdown
```<language>
<code content>
```
````

Where `<language>` is: `python`, `bash`, `yaml`, `xml`, `javascript`, etc.

### Supported Languages

| Language | Use Case | Example |
|----------|----------|---------|
| `python` | ROS 2 nodes, AI planners, utilities | rclpy publisher nodes |
| `bash` | Shell commands, installation steps | `ros2 run` commands |
| `yaml` | Configuration files | Gazebo SDF, Docker Compose |
| `xml` | URDF robot definitions | Humanoid robot description |
| `javascript` | Docusaurus config (rare) | docusaurus.config.js |
| `json` | Data formats, API responses | Isaac Sim scene metadata |

## Python Code Examples

### Header Comment Template

```python
#!/usr/bin/env python3
"""
<One sentence description of what this code does>
Part of Module <N>, Chapter <M>: <Chapter Title>
Tested with: ROS 2 <Version>, Ubuntu <Version>, Python <Version>
"""
```

**Field Specifications**:
- **Shebang** (`#!/usr/bin/env python3`): Required for executable scripts
- **Docstring** (triple-quoted): 1-3 sentences describing purpose
- **Module/Chapter**: Location reference for context (e.g., "Module 1, Chapter 2: Your First ROS 2 Node")
- **Tested with**: Specific versions (ROS 2 Humble/Iron, Ubuntu 22.04, Python 3.10)

### Code Body Requirements

1. **Imports**: Group standard library, third-party, and local imports separately
   ```python
   # Standard library
   import sys
   import time

   # Third-party (ROS 2)
   import rclpy
   from rclpy.node import Node

   # Local imports
   from my_package.utils import helper_function
   ```

2. **Inline Comments**: Explain **why**, not **what**
   ```python
   # Good: Explains why
   self.timer = self.create_timer(1.0, self.callback)  # Publish every 1 second

   # Bad: Explains what (obvious from code)
   self.counter = 0  # Set counter to zero
   ```

3. **Function/Class Docstrings**: For complex functions (optional for simple ones)
   ```python
   def complex_calculation(x, y):
       """
       Calculate trajectory given start and end points.

       Args:
           x (float): Start position in meters
           y (float): End position in meters

       Returns:
           list: Waypoints as (x, y) tuples
       """
       # Implementation
   ```

4. **Main Function**: Always include for runnable scripts
   ```python
   def main(args=None):
       rclpy.init(args=args)
       node = MyNode()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Expected Output

**Always show** expected output after code blocks:

```python
# Your code here

# Expected output:
# [INFO] [timestamp] [node_name]: Message published
# [INFO] [timestamp] [node_name]: Data received: ...
```

**Or** in text following the code block:

When you run this node, you should see:
```
[INFO] Publishing: "Hello ROS 2! Count: 0"
[INFO] Publishing: "Hello ROS 2! Count: 1"
```

### Formatting Standards

- **Indentation**: 4 spaces (not tabs)
- **Line Length**: Max 88 characters (Black formatter default)
- **String Quotes**: Prefer double quotes for strings (consistency)
- **Naming**:
  - Classes: `PascalCase` (e.g., `HelloPublisher`)
  - Functions/Variables: `snake_case` (e.g., `timer_callback`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `MAX_SPEED`)

### Complete Python Example

````markdown
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

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected output (first 3 iterations):
# [INFO] Publishing: "Hello ROS 2! Count: 0"
# [INFO] Publishing: "Hello ROS 2! Count: 1"
# [INFO] Publishing: "Hello ROS 2! Count: 2"
```
````

## Bash Command Examples

### Single Command

````markdown
```bash
ros2 run my_package my_node
```
````

### Multiple Commands

**Sequential** (use `&&` if each depends on previous):
````markdown
```bash
cd ~/ros2_ws
colcon build && source install/setup.bash
```
````

**Independent** (separate blocks or comment which is which):
````markdown
```bash
# Terminal 1: Run the publisher
ros2 run demo_nodes_py talker
```

In a new terminal:
```bash
# Terminal 2: Run the subscriber
ros2 run demo_nodes_py listener
```
````

### Installation Commands

Show full command with explanations:

````markdown
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

**Explanation**: This installs ROS 2 Humble with desktop tools (RViz, rqt, etc.).
````

### Expected Output for Bash

````markdown
```bash
ros2 topic list
```

**Output**:
```
/hello_topic
/rosout
```
````

## YAML/XML Configuration Examples

### YAML (Gazebo, Docker, Config Files)

````markdown
```yaml
# Gazebo world configuration
# Part of Module 2, Chapter 2: Physics Simulation

world:
  name: "simple_world"
  gravity: [0, 0, -9.81]  # m/s^2 in x, y, z
  physics:
    engine: "ode"  # Open Dynamics Engine
    max_step_size: 0.001  # 1 ms timestep for accuracy
```
````

### XML (URDF Robot Descriptions)

````markdown
```xml
<!-- Simple humanoid torso definition -->
<!-- Part of Module 1, Chapter 5: URDF for Humanoids -->

<robot name="simple_humanoid">
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>  <!-- Width, Depth, Height in meters -->
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>  <!-- R G B A -->
      </material>
    </visual>
  </link>
</robot>
```
````

## Code Snippets (Partial Code)

For **partial code** (not full runnable programs), use comments to indicate:

````markdown
```python
# ... (previous code omitted for brevity)

def new_function_to_highlight():
    """This is the new code we're adding."""
    pass

# ... (rest of code continues)
```
````

**Note**: Prefer **full runnable examples** over snippets whenever possible.

## Version Pinning

### ROS 2 Versions

Always specify which ROS 2 version in header comment:
- **ROS 2 Humble** (Ubuntu 22.04, LTS until 2027) - Recommended
- **ROS 2 Iron** (Ubuntu 22.04, released 2023) - Also acceptable

### Python Versions

- **Python 3.10** (Ubuntu 22.04 default) - Standard
- **Python 3.11** - Acceptable if specified

### Other Tools

- **Isaac Sim**: 2023.1.x (latest tested version)
- **Gazebo**: Gazebo Classic 11 or Gazebo Fortress
- **Ubuntu**: 22.04 LTS

## Anti-Patterns (Avoid These)

### ❌ No Header Comment

```python
import rclpy
from rclpy.node import Node
# ... code without context
```

**Problem**: Reader doesn't know what this code does or where it fits in the book.

### ❌ Uncommented Magic Numbers

```python
self.timer = self.create_timer(0.1, self.callback)
```

**Problem**: Why 0.1 seconds? Is that fast or slow?

**Fix**:
```python
self.timer = self.create_timer(0.1, self.callback)  # 10 Hz control loop
```

### ❌ No Expected Output

````markdown
```python
# Code here
```
````

**Problem**: Reader doesn't know if their code is working correctly.

**Fix**: Always show expected output.

### ❌ Non-Runnable Code

```python
# Assume 'node' is already defined somewhere
node.do_something()
```

**Problem**: Reader can't copy-paste and run this.

**Fix**: Provide complete, self-contained examples.

### ❌ Hardcoded Paths

```python
f = open('/home/john/ros2_ws/data.txt', 'r')
```

**Problem**: Won't work on reader's machine.

**Fix**: Use relative paths or environment variables:
```python
import os
ws_path = os.getenv('ROS_WS', os.path.expanduser('~/ros2_ws'))
f = open(os.path.join(ws_path, 'data.txt'), 'r')
```

## Validation

### Automated Checks (Future Enhancement)

- **Syntax Validation**: Python code parses without errors
- **Style Check**: Black formatter compliance
- **Link Check**: Referenced modules/packages exist in ROS 2

### Manual Review Checklist

- [ ] Header comment present with version information
- [ ] Inline comments explain non-obvious logic
- [ ] Expected output included
- [ ] Code is copy-paste runnable
- [ ] Formatting follows standards (4-space indent, 88-char lines)
- [ ] No hardcoded user-specific paths
- [ ] Version pinning explicit (ROS 2 Humble, Python 3.10)

## Summary

**Golden Rule**: Every code example must be **runnable by a reader** with the specified environment (ROS 2 Humble, Ubuntu 22.04, Python 3.10).

**Priorities**:
1. **Correctness**: Code works when copy-pasted
2. **Clarity**: Comments explain why, not what
3. **Context**: Header shows where it fits in the book
4. **Output**: Reader knows what success looks like
