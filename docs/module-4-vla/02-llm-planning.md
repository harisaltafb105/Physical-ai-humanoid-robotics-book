---
id: llm-planning
title: 'Chapter 2: LLM Cognitive Planning'
sidebar_position: 2
last_updated: 2025-12-06
---

# Chapter 2: LLM Cognitive Planning

## Learning Objectives

- Use LLMs (GPT-4, Claude) for high-level task planning
- Implement "Code as Policies" for robot control
- Apply prompt engineering techniques for robotics
- Handle failures and replan dynamically
- Integrate safety constraints and error checking

**Reading time**: 12 minutes

## LLMs as Robot Planners

### Traditional Planning vs. LLM Planning

**Traditional** (symbolic planning):
```python
# Hardcoded rules
if task == "clean_table":
    steps = ["find_sponge", "wipe_surface", "put_away_items"]
```

**LLM-based** (emergent reasoning):
```text
Human: "Clean the kitchen table"
GPT-4: "1. Check if table has dishes → move to sink
        2. Get sponge from drawer
        3. Wipe table surface
        4. Put sponge back"
```

**Advantage**: LLM generalizes to new tasks without explicit programming.

## Code as Policies

### Concept

**Idea**: LLM writes Python code to control robot, not just text plans.

**Example prompt**:
```text
User: "Stack the blue block on top of the red block"

LLM generates:
```python
# Robot control code
blue_block = find_object("blue", "block")
red_block = find_object("red", "block")
grasp(blue_block)
move_above(red_block, height=0.1)
release()
```

### Implementation

```python
#!/usr/bin/env python3
"""
Code as Policies: GPT-4 writes robot control code
Part of Module 4, Chapter 2
Tested with: OpenAI API, ROS 2 Humble, Python 3.10
"""

import openai
import os

# Set API key
openai.api_key = os.getenv("OPENAI_API_KEY")

def generate_robot_code(task_description):
    """Use GPT-4 to generate robot control code"""

    prompt = f"""
You are a robot control programmer. Given a task description, write Python code
to control a robot with these functions available:

- find_object(color, type) → returns object ID
- grasp(object_id) → closes gripper on object
- move_to(x, y, z) → moves robot end-effector
- release() → opens gripper

Task: {task_description}

Write Python code to complete this task. Include error checking.
"""

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a robotics expert."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.2,  # Low temperature for deterministic output
        max_tokens=500
    )

    code = response['choices'][0]['message']['content']
    return code

# Example usage
task = "Pick up the red mug and place it on the table"
generated_code = generate_robot_code(task)

print("Generated Code:")
print(generated_code)

# Expected output (GPT-4's response):
# ```python
# # Find the red mug
# red_mug = find_object("red", "mug")
# if red_mug is None:
#     print("Error: Red mug not found")
#     return False
#
# # Grasp the mug
# grasp_success = grasp(red_mug)
# if not grasp_success:
#     print("Error: Failed to grasp mug")
#     return False
#
# # Move to table position
# table_pos = (0.5, 0.3, 0.2)  # x, y, z in meters
# move_to(*table_pos)
#
# # Release mug
# release()
# print("Success: Mug placed on table")
# return True
# ```
```

## Prompt Engineering for Robotics

### Effective Prompt Structure

**1. Context**: Define robot capabilities
**2. Examples**: Few-shot learning (show examples)
**3. Constraints**: Safety rules
**4. Task**: Specific goal

### Example Prompt Template

```python
ROBOT_PROMPT = """
You are planning actions for a humanoid robot with these capabilities:

AVAILABLE ACTIONS:
- navigate(location) → moves robot to location
- detect_objects() → returns list of visible objects
- grasp(object) → picks up object
- place(object, location) → puts object at location
- speak(text) → robot says text

CONSTRAINTS:
- Robot can only hold one object at a time
- Robot cannot reach above 2 meters
- Robot must avoid collisions

EXAMPLES:
Task: "Bring me the book from the shelf"
Plan:
1. navigate("shelf")
2. objects = detect_objects()
3. book = find_book(objects)
4. grasp(book)
5. navigate("user")
6. place(book, "user_hand")

Your task: {user_command}
Generate a step-by-step plan using only the available actions.
"""

def plan_task(user_command):
    prompt = ROBOT_PROMPT.format(user_command=user_command)

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.0
    )

    plan = response['choices'][0]['message']['content']
    return plan

# Example
plan = plan_task("Find the red box and bring it to the kitchen")
print(plan)

# Expected output:
# 1. current_room = get_current_location()
# 2. objects = detect_objects()
# 3. red_box = find_object(objects, color="red", type="box")
# 4. if red_box is None:
#      navigate("living_room")
#      objects = detect_objects()
#      red_box = find_object(objects, color="red", type="box")
# 5. grasp(red_box)
# 6. navigate("kitchen")
# 7. place(red_box, "kitchen_counter")
# 8. speak("I brought the red box to the kitchen")
```

## Integrating LLM with ROS 2

### Complete LLM Planning Node

```python
#!/usr/bin/env python3
"""
LLM-based task planner integrated with ROS 2
Part of Module 4, Chapter 2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import openai
import os
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        openai.api_key = os.getenv("OPENAI_API_KEY")

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )

        # Publish robot actions
        self.action_pub = self.create_publisher(String, '/robot_action', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("LLM Planner ready. Waiting for commands...")

    def command_callback(self, msg):
        """Receive voice command and generate plan"""
        task = msg.data
        self.get_logger().info(f"📝 Task: {task}")

        # Generate plan with GPT-4
        plan = self.generate_plan(task)
        self.get_logger().info(f"🧠 LLM Plan:\n{plan}")

        # Execute plan
        self.execute_plan(plan)

    def generate_plan(self, task):
        """Use LLM to create step-by-step plan"""

        prompt = f"""
Task: {task}

Available robot actions (ROS 2 topics/services):
- publish_velocity(linear_x, angular_z) → move robot
- detect_objects() → returns list of objects
- speak(text) → text-to-speech

Generate a JSON list of actions to complete the task.
Format: [{{"action": "action_name", "params": {{}}}}, ...]
"""

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.0
        )

        plan_text = response['choices'][0]['message']['content']

        # Extract JSON from response
        import re
        json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
        if json_match:
            plan = json.loads(json_match.group(0))
            return plan
        else:
            return []

    def execute_plan(self, plan):
        """Execute each step in the plan"""

        for step in plan:
            action = step.get("action")
            params = step.get("params", {})

            self.get_logger().info(f"Executing: {action} with params {params}")

            if action == "publish_velocity":
                twist = Twist()
                twist.linear.x = params.get("linear_x", 0.0)
                twist.angular.z = params.get("angular_z", 0.0)
                self.cmd_pub.publish(twist)

                # Execute for duration
                import time
                time.sleep(params.get("duration", 2.0))

                # Stop
                stop_msg = Twist()
                self.cmd_pub.publish(stop_msg)

            elif action == "speak":
                text = params.get("text", "")
                self.get_logger().info(f"🔊 Speaking: {text}")
                # In real system: call text-to-speech service

            elif action == "detect_objects":
                # In real system: call vision service
                self.get_logger().info("👁️ Detecting objects...")

            # Publish action for monitoring
            action_msg = String()
            action_msg.data = f"{action}: {params}"
            self.action_pub.publish(action_msg)

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Example usage:
# ros2 topic pub /voice_command std_msgs/String "data: 'move forward 2 meters and turn right'"
#
# Expected output:
# [INFO] 📝 Task: move forward 2 meters and turn right
# [INFO] 🧠 LLM Plan:
# [{"action": "publish_velocity", "params": {"linear_x": 0.5, "angular_z": 0.0, "duration": 4.0}},
#  {"action": "publish_velocity", "params": {"linear_x": 0.0, "angular_z": -0.5, "duration": 3.14}}]
# [INFO] Executing: publish_velocity with params {'linear_x': 0.5, 'angular_z': 0.0, 'duration': 4.0}
# [INFO] Executing: publish_velocity with params {'linear_x': 0.0, 'angular_z': -0.5, 'duration': 3.14}
```

## Error Handling and Replanning

### Inner Monologue Approach

When robot fails, LLM reasons about why and generates new plan.

```python
def replan_on_failure(original_task, error_message):
    """LLM replans when execution fails"""

    prompt = f"""
The robot was trying to: {original_task}

But encountered this error: {error_message}

Analyze what went wrong and generate a new plan that avoids the failure.
"""

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.3  # Slightly higher for creative problem-solving
    )

    new_plan = response['choices'][0]['message']['content']
    return new_plan

# Example
original = "Pick up the red mug"
error = "Error: Gripper failed to close. Object may be too large."

new_plan = replan_on_failure(original, error)
print(new_plan)

# Expected output:
# "The mug may be too large for the gripper. Alternative plan:
#  1. Measure object dimensions using depth camera
#  2. If width > gripper_max_width, use two-handed grasp
#  3. Approach from different angle (top-down vs. side)
#  4. Reduce gripper force to avoid slipping"
```

## Safety Constraints

### Filtering Dangerous Commands

```python
def is_safe_action(action, params):
    """Verify action is safe before execution"""

    # Check velocity limits
    if action == "publish_velocity":
        if abs(params.get("linear_x", 0)) > 2.0:  # Max 2 m/s
            return False, "Velocity too high"
        if abs(params.get("angular_z", 0)) > 1.5:  # Max 1.5 rad/s
            return False, "Angular velocity too high"

    # Check workspace limits
    if action == "move_to":
        x, y, z = params.get("x"), params.get("y"), params.get("z")
        if z > 2.0:  # Cannot reach above 2m
            return False, "Target too high"
        if x < -1.0 or x > 1.0:  # Workspace bounds
            return False, "Target outside workspace"

    # Prohibit dangerous commands
    forbidden_keywords = ["fire", "explosion", "harm", "attack"]
    action_str = str(action).lower()
    if any(word in action_str for word in forbidden_keywords):
        return False, "Dangerous action prohibited"

    return True, "Safe"

# Example
safe, reason = is_safe_action("publish_velocity", {"linear_x": 5.0})
print(f"Safe: {safe}, Reason: {reason}")
# Output: Safe: False, Reason: Velocity too high
```

## Summary

In this chapter, you learned:

✅ **LLM planning**: GPT-4 decomposes tasks into step-by-step plans
✅ **Code as Policies**: LLMs generate executable robot code
✅ **Prompt engineering**: Structured prompts for robust planning
✅ **Error handling**: Replanning when execution fails
✅ **Safety**: Constraint checking and forbidden action filtering

**Key takeaway**: LLMs bring human-like reasoning to robots. With proper prompts and safety checks, they can plan complex, multi-step tasks.

## Next Steps

In [Chapter 3: Integration & Case Studies](./03-integration-case-studies.md), you'll see:
- End-to-end VLA architectures combining vision + LLM + actions
- Real-world deployments (household robots, warehouses, hospitals)
- Future directions (multimodal LLMs, world models, embodied foundation models)

---

**Hands-on challenge**: Create a prompt that makes GPT-4 generate a plan for "organize the bookshelf by color". Test it and see if the LLM correctly sequences: detect books → group by color → rearrange.
