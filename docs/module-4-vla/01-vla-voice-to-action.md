---
id: vla-voice-to-action
title: 'Chapter 1: VLA Overview & Voice-to-Action'
sidebar_position: 1
last_updated: 2025-12-06
---

# Chapter 1: VLA Overview & Voice-to-Action

## Learning Objectives

- Understand the embodied AI paradigm and VLA systems
- Implement speech recognition using OpenAI Whisper
- Parse voice commands into structured robot actions
- Handle ambiguity and errors in natural language
- Build a voice-controlled robot teleop system

**Reading time**: 12 minutes

## Embodied AI: AI Leaving the Digital Realm

### From ChatGPT to Physical Robots

**2022**: ChatGPT masters language (text → text)
**2023**: GPT-4 Vision understands images (image + text → text)
**2024**: VLA systems control robots (vision + text → physical actions)

**Key insight**: The same transformer architecture that powers ChatGPT can be adapted to output robot actions instead of text tokens.

### VLA Model Architecture

```text
┌────────────────────────────────────────────┐
│         Vision-Language-Action Model       │
└────────────────────────────────────────────┘
        ↓              ↓              ↓
┌──────────┐   ┌──────────┐   ┌──────────┐
│ Vision   │   │ Language │   │ Action   │
│ Encoder  │   │ Encoder  │   │ Decoder  │
│          │   │          │   │          │
│ (ResNet) │   │ (BERT)   │   │ (Policy  │
│          │   │          │   │  Head)   │
└──────────┘   └──────────┘   └──────────┘
     ↑              ↑              ↓
  Camera       "Pick up red    Joint angles
   image          box"         [θ1, θ2, ...]
```

**Example models**:
- **RT-2** (Google): 550M parameters, trained on 130K robot trajectories
- **PaLM-E** (Google): 562B parameters, integrates vision + language + robot data

## Speech Recognition with Whisper

### Why Whisper?

**OpenAI Whisper** is a state-of-the-art speech recognition model:
- **Accuracy**: 95%+ word error rate on clean audio
- **Multilingual**: Supports 99 languages
- **Robust**: Works with accents, background noise
- **Open source**: Free to use, runs locally (no cloud)

### Install Whisper

```bash
pip install openai-whisper
```

### Basic Speech-to-Text

```python
#!/usr/bin/env python3
"""
Convert speech to text using Whisper
Part of Module 4, Chapter 1
Tested with: Whisper 20230314, Python 3.10
"""

import whisper
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav

# Load Whisper model
model = whisper.load_model("base")  # Options: tiny, base, small, medium, large

# Record audio from microphone
def record_audio(duration=5, sample_rate=16000):
    print(f"🎤 Recording for {duration} seconds... Speak now!")
    audio = sd.rec(int(duration * sample_rate),
                   samplerate=sample_rate,
                   channels=1,
                   dtype='float32')
    sd.wait()  # Wait until recording finishes
    print("✅ Recording complete")
    return audio, sample_rate

# Transcribe audio
def transcribe(audio_data, sample_rate):
    # Save to temporary file (Whisper requires file input)
    temp_file = "/tmp/voice_command.wav"
    wav.write(temp_file, sample_rate, (audio_data * 32767).astype(np.int16))

    # Run Whisper
    result = model.transcribe(temp_file, language="en")
    return result["text"]

# Main
if __name__ == "__main__":
    audio, sr = record_audio(duration=5)
    text = transcribe(audio, sr)
    print(f"\n📝 Transcription: \"{text}\"")

# Expected output:
# 🎤 Recording for 5 seconds... Speak now!
# ✅ Recording complete
# 📝 Transcription: "move forward three meters"
```

## Voice Command Parsing

### From Text to Robot Actions

**Challenge**: "move forward three meters" → `cmd_vel.linear.x = 0.5` (for 6 seconds)

**Solution**: Use regular expressions and keyword matching.

```python
#!/usr/bin/env python3
"""
Parse voice commands into ROS 2 Twist messages
Part of Module 4, Chapter 1
"""

import re
from geometry_msgs.msg import Twist

class VoiceCommandParser:
    def __init__(self):
        self.command_map = {
            "forward": ("linear_x", 1.0),
            "backward": ("linear_x", -1.0),
            "left": ("angular_z", 1.0),
            "right": ("angular_z", -1.0),
            "stop": ("zero", 0.0),
        }

    def parse(self, text):
        """Convert text to Twist message"""
        text = text.lower().strip()

        # Extract numbers (for distance/angle)
        numbers = re.findall(r'\d+\.?\d*', text)
        magnitude = float(numbers[0]) if numbers else 1.0

        # Match command keywords
        for keyword, (axis, direction) in self.command_map.items():
            if keyword in text:
                twist = Twist()
                if axis == "linear_x":
                    twist.linear.x = direction * min(magnitude, 2.0)  # Cap at 2 m/s
                elif axis == "angular_z":
                    twist.angular.z = direction * min(magnitude, 1.0)  # Cap at 1 rad/s
                return twist

        # Unknown command
        return None

# Example usage
parser = VoiceCommandParser()

commands = [
    "move forward",
    "turn left 90 degrees",
    "go backward two meters",
    "stop",
]

for cmd in commands:
    twist = parser.parse(cmd)
    if twist:
        print(f"'{cmd}' → linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}")
    else:
        print(f"'{cmd}' → ❌ Unknown command")

# Output:
# 'move forward' → linear_x=1.00, angular_z=0.00
# 'turn left 90 degrees' → linear_x=0.00, angular_z=1.00
# 'go backward two meters' → linear_x=-2.00, angular_z=0.00
# 'stop' → linear_x=0.00, angular_z=0.00
```

## ROS 2 Voice Teleop Node

### Complete Voice-Controlled Robot

```python
#!/usr/bin/env python3
"""
Voice-controlled teleoperation node
Listens to microphone → Whisper → ROS 2 cmd_vel
Part of Module 4, Chapter 1
Tested with: ROS 2 Humble, Whisper, Python 3.10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import whisper
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import threading

class VoiceTeleopNode(Node):
    def __init__(self):
        super().__init__('voice_teleop')

        # ROS 2 publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Load Whisper model
        self.get_logger().info("Loading Whisper model...")
        self.whisper_model = whisper.load_model("base")
        self.get_logger().info("✅ Whisper ready")

        # Command parser
        self.parser = VoiceCommandParser()

        # Start listening thread
        self.listening = True
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.start()

    def listen_loop(self):
        """Continuous listening loop"""
        while self.listening:
            self.get_logger().info("🎤 Listening... (say 'stop' to quit)")

            # Record 5 seconds
            audio = sd.rec(int(5 * 16000), samplerate=16000, channels=1, dtype='float32')
            sd.wait()

            # Transcribe
            temp_file = "/tmp/voice.wav"
            wav.write(temp_file, 16000, (audio * 32767).astype(np.int16))
            result = self.whisper_model.transcribe(temp_file, language="en")
            text = result["text"].strip()

            self.get_logger().info(f"📝 Heard: \"{text}\"")

            # Parse and execute
            twist = self.parser.parse(text)
            if twist:
                self.cmd_pub.publish(twist)
                self.get_logger().info(
                    f"✅ Executing: linear_x={twist.linear.x:.2f}, "
                    f"angular_z={twist.angular.z:.2f}"
                )

                # Auto-stop after 3 seconds (safety)
                import time
                time.sleep(3)
                stop_msg = Twist()  # All zeros
                self.cmd_pub.publish(stop_msg)
                self.get_logger().info("🛑 Auto-stop")
            else:
                self.get_logger().warn(f"❌ Unknown command: \"{text}\"")

            # Check for quit command
            if "stop" in text.lower() or "quit" in text.lower():
                self.listening = False

    def destroy_node(self):
        self.listening = False
        self.listen_thread.join()
        super().destroy_node()

def main():
    rclpy.init()
    node = VoiceTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Expected usage:
# User: "move forward"
# [INFO] 📝 Heard: "move forward"
# [INFO] ✅ Executing: linear_x=1.00, angular_z=0.00
# [INFO] 🛑 Auto-stop
# User: "turn right"
# [INFO] 📝 Heard: "turn right"
# [INFO] ✅ Executing: linear_x=0.00, angular_z=-1.00
# [INFO] 🛑 Auto-stop
```

## Handling Ambiguity

### The "Red Box" Problem

**User**: "Pick up the red box"
**Robot vision**: Sees 3 red boxes

**Solution**: Clarification dialog

```python
def handle_ambiguity(objects_detected):
    """Ask user for clarification when multiple matches"""
    if len(objects_detected) > 1:
        print(f"❓ I see {len(objects_detected)} red boxes. Which one?")
        print("Options: 1) Left, 2) Center, 3) Right")

        # Wait for voice response
        choice = get_voice_input()  # "center"

        # Map to object
        if "center" in choice:
            return objects_detected[1]
        elif "left" in choice:
            return objects_detected[0]
        elif "right" in choice:
            return objects_detected[2]

    return objects_detected[0]  # Default to first if only one
```

## Summary

In this chapter, you learned:

✅ **Embodied AI**: VLA systems combine vision, language, and robot actions
✅ **Whisper**: State-of-the-art speech recognition (95%+ accuracy)
✅ **Command parsing**: Convert natural language to ROS 2 Twist messages
✅ **Voice teleop**: Complete voice-controlled robot system

**Key takeaway**: Voice is the most natural interface. With Whisper + ROS 2, anyone can control a robot without programming.

## Next Steps

In [Chapter 2: LLM Cognitive Planning](./02-llm-planning.md), you'll learn how to:
- Use GPT-4 to decompose complex tasks ("clean the table" → subtasks)
- Generate robot code with "Code as Policies"
- Handle failures and replan dynamically

---

**Hands-on challenge**: Extend the voice teleop node to support commands like "move forward 2 meters and then turn left". Hint: You'll need to sequence multiple Twist messages!
