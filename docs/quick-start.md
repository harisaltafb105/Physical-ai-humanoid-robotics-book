---
id: quick-start
title: Quick Start Guide
sidebar_position: 2
last_updated: 2025-12-06
---

# Quick Start: Your First ROS 2 Robot in 30 Minutes

This guide will take you from zero to running your first simulated humanoid robot in just 30 minutes. By the end, you'll have hands-on experience with ROS 2 nodes, topics, and basic robot control.

## Prerequisites

Before starting, ensure you have:

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04
- **Hardware**: Computer with at least 4GB RAM and 20GB free disk space
- **Time**: 30 minutes focused learning time

:::tip Don't have Ubuntu?
You can use Windows Subsystem for Linux (WSL2) or a virtual machine. See [Alternative Setups](#alternative-setups) at the end of this guide.
:::

## Step 1: Install ROS 2 Humble (10 minutes)

ROS 2 Humble is the Long-Term Support (LTS) version we'll use throughout this book.

### 1.1 Set Up Sources

```bash
# Ensure UTF-8 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.2 Install ROS 2 Packages

```bash
# Update package index
sudo apt update

# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

**Expected output**: Installation completes without errors (3-7 minutes depending on internet speed)

### 1.3 Source the Setup Script

```bash
# Source ROS 2 environment (add to ~/.bashrc for automatic loading)
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
```

**Expected output**:
```
ros2 cli version: 0.18.9
```

:::tip Automatic Environment Setup
Add `source /opt/ros/humble/setup.bash` to your `~/.bashrc` file so ROS 2 is available in every terminal:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
:::

## Step 2: Create Your First ROS 2 Node (5 minutes)

Let's create a simple "Hello World" publisher node in Python.

### 2.1 Create a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_quickstart/src
cd ~/ros2_quickstart
```

### 2.2 Write the Publisher Node

Create a file `~/ros2_quickstart/hello_publisher.py`:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 publisher node - your first robot program!
Part of Quick Start Guide
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        # Create publisher on 'hello_topic' with queue size 10
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        # Publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Hello Publisher started!')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Physical AI! Message #{self.counter}'
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
```

### 2.3 Make it Executable and Run

```bash
# Make the script executable
chmod +x ~/ros2_quickstart/hello_publisher.py

# Run your first ROS 2 node!
python3 ~/ros2_quickstart/hello_publisher.py
```

**Expected output**:
```
[INFO] [hello_publisher]: Hello Publisher started!
[INFO] [hello_publisher]: Publishing: "Hello Physical AI! Message #0"
[INFO] [hello_publisher]: Publishing: "Hello Physical AI! Message #1"
[INFO] [hello_publisher]: Publishing: "Hello Physical AI! Message #2"
...
```

Press `Ctrl+C` to stop the node.

🎉 **Congratulations!** You just created and ran your first ROS 2 node!

## Step 3: Explore ROS 2 Command-Line Tools (5 minutes)

ROS 2 provides powerful CLI tools to inspect running nodes and topics.

### 3.1 List Active Topics

Keep your publisher running in one terminal, then open a new terminal:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# List all active topics
ros2 topic list
```

**Expected output**:
```
/hello_topic
/parameter_events
/rosout
```

### 3.2 Echo Topic Messages

See the messages your node is publishing in real-time:

```bash
ros2 topic echo /hello_topic
```

**Expected output**:
```
data: 'Hello Physical AI! Message #15'
---
data: 'Hello Physical AI! Message #16'
---
...
```

### 3.3 Inspect Topic Information

```bash
# See topic type and publisher count
ros2 topic info /hello_topic
```

**Expected output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### 3.4 View Topic Publishing Rate

```bash
# Check how fast messages are being published
ros2 topic hz /hello_topic
```

**Expected output**:
```
average rate: 1.000
  min: 0.999s max: 1.001s std dev: 0.00071s window: 10
```

Press `Ctrl+C` to stop monitoring.

## Step 4: Create a Subscriber (5 minutes)

Now let's create a node that listens to your publisher.

### 4.1 Write the Subscriber Node

Create `~/ros2_quickstart/hello_subscriber.py`:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 subscriber node - listens to publisher.
Part of Quick Start Guide
Tested with: ROS 2 Humble, Ubuntu 22.04, Python 3.10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloSubscriber(Node):
    def __init__(self):
        super().__init__('hello_subscriber')
        # Subscribe to 'hello_topic' with queue size 10
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('Hello Subscriber started! Waiting for messages...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloSubscriber()
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

### 4.2 Run Publisher and Subscriber Together

**Terminal 1** (Publisher):
```bash
python3 ~/ros2_quickstart/hello_publisher.py
```

**Terminal 2** (Subscriber):
```bash
chmod +x ~/ros2_quickstart/hello_subscriber.py
python3 ~/ros2_quickstart/hello_subscriber.py
```

**Expected output in Terminal 2**:
```
[INFO] [hello_subscriber]: Hello Subscriber started! Waiting for messages...
[INFO] [hello_subscriber]: Received: "Hello Physical AI! Message #8"
[INFO] [hello_subscriber]: Received: "Hello Physical AI! Message #9"
...
```

🚀 **You're now running a multi-node ROS 2 system!** This is the foundation of all robot software.

## Step 5: Visualize with RQT (5 minutes)

RQT is ROS 2's graphical tool suite for monitoring and debugging.

### 5.1 Launch RQT Graph

With both nodes running, open a third terminal:

```bash
# Launch node graph visualizer
rqt_graph
```

A window will open showing:
- Two nodes: `/hello_publisher` and `/hello_subscriber`
- One topic: `/hello_topic` connecting them
- Message flow direction (publisher → topic → subscriber)

### 5.2 Launch RQT Topic Monitor

```bash
# View all topics and their data
rqt
```

In the RQT window:
1. Click **Plugins** → **Topics** → **Topic Monitor**
2. Check the box next to `/hello_topic`
3. You'll see messages updating in real-time!

## What You've Learned

In just 30 minutes, you've:

✅ Installed ROS 2 Humble (the industry-standard robot middleware)
✅ Created a publisher node that sends messages
✅ Created a subscriber node that receives messages
✅ Used ROS 2 CLI tools (`topic list`, `echo`, `info`, `hz`)
✅ Visualized your robot system with RQT
✅ Understood the **node-topic-message** architecture that powers all ROS 2 robots

## Next Steps

### Continue Your Learning Journey

Now that you understand ROS 2 fundamentals, proceed to:

1. **[Module 1: ROS 2 Fundamentals](./module-1-ros2/index.md)** - Deep dive into nodes, topics, services, and URDF
2. **[Module 2: Gazebo Simulation](./module-2-gazebo/index.md)** - Simulate physics-based robots
3. **[Module 3: NVIDIA Isaac](./module-3-isaac/index.md)** - Photorealistic simulation and AI perception
4. **[Module 4: Vision-Language-Action](./module-4-vla/index.md)** - Add voice and LLM planning

### Experiment on Your Own

Try modifying the code:

- Change the publishing rate from 1 Hz to 0.5 Hz (every 2 seconds)
- Modify the message content to include a timestamp
- Create a third node that subscribes and republishes with a prefix
- Add a service that the subscriber can call to reset the publisher's counter

### Get Help

- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **ROS 2 Discourse**: https://discourse.ros.org/
- **Glossary**: Check our [Glossary](./glossary.md) for ROS 2 term definitions

## Troubleshooting

### Issue: "ros2: command not found"

**Solution**: Source the ROS 2 setup script:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: "ModuleNotFoundError: No module named 'rclpy'"

**Solution**: Install rclpy:
```bash
sudo apt install python3-rclpy
```

### Issue: Publisher and Subscriber can't communicate

**Solution**: Ensure both nodes are using the same ROS domain:
```bash
# Check ROS domain (should be 0 by default)
echo $ROS_DOMAIN_ID
```

### Issue: Permission denied when running script

**Solution**: Make the script executable:
```bash
chmod +x ~/ros2_quickstart/hello_publisher.py
```

## Alternative Setups

### Windows (WSL2)

1. Install WSL2 with Ubuntu 22.04:
```powershell
wsl --install -d Ubuntu-22.04
```

2. Follow the same installation steps inside WSL

3. For GUI tools (RQT), install an X server like VcXsrv

### macOS

ROS 2 Humble doesn't officially support macOS. We recommend:
- Using Docker with Ubuntu 22.04 container
- Running a Ubuntu VM with VirtualBox or Parallels

### Docker (Any OS)

```bash
# Pull ROS 2 Humble image
docker pull osrf/ros:humble-desktop

# Run container with X11 forwarding
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop
```

---

**You're now ready to build intelligent robots!** Proceed to [Module 1](./module-1-ros2/index.md) to deepen your understanding of ROS 2 architecture.
