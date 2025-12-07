# Implementation Plan: Physical AI & Humanoid Robotics Book

**Branch**: `001-physical-ai-robotics-book` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-robotics-book/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive, concise, and engaging educational book on Physical AI and Humanoid Robotics. The book will cover 4 progressive modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action) deployed as a Docusaurus website to GitHub Pages. Target reading time: 4-6 hours. Success metrics include 30-minute quick start, responsive design, and single-command deployment.

**Technical Approach**: Use Docusaurus 3.x static site generator with MDX for interactive content. Organize as hierarchical documentation with sidebar navigation, search, and code highlighting. Deploy via GitHub Actions to GitHub Pages. Content-first development prioritizing learning outcomes over technical complexity.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+) for Docusaurus, Markdown/MDX for content
**Primary Dependencies**: Docusaurus 3.x, React 18+, MDX 3.x, Prism (syntax highlighting), Algolia DocSearch (search)
**Storage**: Static markdown files in Git repository, no database required
**Testing**: Markdown linting (markdownlint), link checking (broken-link-checker), build validation (Docusaurus build)
**Target Platform**: Static website deployed to GitHub Pages, accessible via modern browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Documentation website (Docusaurus-based static site)
**Performance Goals**: < 3 second initial load, < 1 second navigation between pages, searchable 4-6 hour content corpus
**Constraints**: Must build in < 5 minutes, deploy automatically on push, support mobile/tablet/desktop responsive layout
**Scale/Scope**: ~40-60 markdown pages (4 modules × 8-12 chapters each + intro/hardware/glossary), ~150-200 code examples

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Content-First Development ✅ PASS
- Plan prioritizes content structure and learning objectives before technical implementation
- research.md will define module/chapter outlines with learning outcomes
- data-model.md will specify content entities (Module, Chapter, Code Example) before file structure

### Principle II: Docusaurus Framework Adherence ✅ PASS
- Using Docusaurus 3.x following official best practices
- Configuration files (docusaurus.config.js, sidebars.js) will follow Docusaurus conventions
- MDX features (admonitions, tabs, code blocks) used appropriately per Docusaurus docs

### Principle III: GitHub Pages Deployment Readiness ✅ PASS
- GitHub Actions workflow will automate build and deployment
- Proper baseUrl and organizationName configuration for GitHub Pages
- Every commit on main branch deployable (continuous deployment)

### Principle IV: Progressive Content Structure ✅ PASS
- 4 modules with clear prerequisite chain: Module 1 → Module 2 → Module 3 → Module 4
- Sidebar navigation supports both linear (prev/next) and non-linear (jump to any chapter) reading
- Learning objectives progress from foundational (ROS 2 basics) to advanced (VLA integration)

### Principle V: Simplicity and Clarity ✅ PASS
- Content target: 4-6 hour reading time (concise, not encyclopedic)
- Code examples: minimal, runnable, heavily commented
- Each markdown file = one chapter with single clear purpose
- No unnecessary build complexity (standard Docusaurus, no custom webpack configs)

### Principle VI: Version Control and Traceability ✅ PASS
- All content in Git with descriptive commits
- Frontmatter in each markdown file includes last-updated date
- Software version requirements explicitly stated (ROS 2 Humble/Iron, Isaac Sim version, Ubuntu 22.04)

**Gate Status**: ✅ **ALL PRINCIPLES SATISFIED** - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-robotics-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0: Content structure and module outlines
├── data-model.md        # Phase 1: Book entities and metadata schemas
├── quickstart.md        # Phase 1: Development environment setup guide
└── contracts/           # Phase 1: Content schemas and structure specifications
    ├── module-schema.md        # Module structure specification
    ├── chapter-schema.md       # Chapter frontmatter and content structure
    └── code-example-schema.md  # Code snippet formatting standards
```

### Source Code (repository root)

```text
# Docusaurus documentation website structure

# Docusaurus configuration
docusaurus.config.js     # Site config, GitHub Pages settings, theme, plugins
sidebars.js              # Sidebar navigation structure for all modules
package.json             # Node.js dependencies (Docusaurus, plugins, tools)

# Content (markdown/MDX files)
docs/
├── intro.md                           # Homepage: What is Physical AI?
├── quick-start.md                     # 30-minute getting started guide
├── hardware/
│   ├── workstation-requirements.md   # RTX GPU, CPU, RAM, Ubuntu specs
│   ├── edge-compute-kit.md           # Jetson Orin, RealSense, IMU
│   └── robot-lab-options.md          # 3 budget tiers (Proxy/Miniature/Premium)
├── module-1-ros2/
│   ├── index.md                       # Module 1 overview and learning objectives
│   ├── 01-ros2-fundamentals.md       # What is ROS 2, nodes, topics
│   ├── 02-first-ros2-node.md         # Hands-on: Create and run your first node
│   ├── 03-topics-and-services.md     # Communication patterns in ROS 2
│   ├── 04-python-rclpy.md            # Bridging Python agents to ROS controllers
│   └── 05-urdf-humanoids.md          # Robot description format for humanoids
├── module-2-digital-twin/
│   ├── index.md                       # Module 2 overview
│   ├── 01-gazebo-intro.md            # Physics simulation basics
│   ├── 02-gazebo-physics.md          # Gravity, collisions, rigid body dynamics
│   ├── 03-unity-rendering.md         # High-fidelity rendering and HRI
│   ├── 04-sensor-simulation.md       # LiDAR, depth cameras, IMUs in simulation
│   └── 05-gazebo-ros2-bridge.md      # Connecting Gazebo to ROS 2 nodes
├── module-3-nvidia-isaac/
│   ├── index.md                       # Module 3 overview
│   ├── 01-isaac-sim-intro.md         # Photorealistic simulation overview
│   ├── 02-isaac-sim-setup.md         # Installing and configuring Isaac Sim
│   ├── 03-synthetic-data.md          # Generating training data in Isaac Sim
│   ├── 04-isaac-ros.md               # Hardware-accelerated perception pipelines
│   ├── 05-vslam.md                   # Visual SLAM with Isaac ROS
│   └── 06-nav2-bipedal.md            # Path planning for humanoid movement
├── module-4-vla/
│   ├── index.md                       # Module 4 overview
│   ├── 01-vla-overview.md            # Convergence of LLMs and robotics
│   ├── 02-voice-to-action.md         # OpenAI Whisper for voice commands
│   ├── 03-llm-planning.md            # Using LLMs for cognitive planning
│   ├── 04-vla-integration.md         # Integrating VLA with ROS 2 robots
│   └── 05-case-studies.md            # Real-world VLA robotics applications
└── glossary.md                        # Technical terms (ROS 2, URDF, VSLAM, VLA, etc.)

# Static assets
static/
├── img/                               # Diagrams, screenshots, architecture visuals
│   ├── ros2-node-diagram.svg         # ROS 2 communication architecture
│   ├── gazebo-physics-pipeline.svg   # Physics simulation workflow
│   ├── isaac-sim-pipeline.svg        # Isaac Sim data generation flow
│   └── vla-architecture.svg          # Vision-Language-Action system diagram
└── favicon.ico                        # Site favicon

# Build output (gitignored)
build/                                 # Generated static site for deployment

# GitHub Actions
.github/
└── workflows/
    └── deploy.yml                     # Automated build and deploy to GitHub Pages

# Development
node_modules/                          # Dependencies (gitignored)
.gitignore                             # Ignore build artifacts, node_modules
README.md                              # Project overview, local dev instructions
```

**Structure Decision**: Selected Docusaurus documentation website structure. This is a specialized "documentation project" variant optimized for educational content. Structure supports:
- **Progressive learning**: 4 modules with clear hierarchy (docs/module-N/)
- **Quick access**: Dedicated quick-start.md and hardware/ section
- **Reference material**: Glossary and searchable content
- **Deployment**: GitHub Actions builds static site from docs/ and deploys to GitHub Pages

## Complexity Tracking

*No constitution violations - table not required.*

All principles satisfied with standard Docusaurus patterns. No complexity justification needed.

---

# Phase 0: Content Structure Research

## Content Organization Strategy

### Module Breakdown (Revised to 3.4 hour concise reading time)

**Reading Time Allocation** (Revised):
- Introduction + Quick Start: 30 minutes
- Hardware Guidance: 30 minutes
- Module 1 (ROS 2): 36 minutes (3 chapters × 12 min each)
- Module 2 (Gazebo): 24 minutes (2 chapters × 12 min each)
- Module 3 (NVIDIA Isaac): 48 minutes (4 chapters × 12 min each)
- Module 4 (VLA): 36 minutes (3 chapters × 12 min each)
- Glossary + Reference: 18 minutes
- **Total: ~3.4 hours** (fits 3-4 hour concise target)

### Module 1: The Robotic Nervous System (ROS 2)

**Learning Objectives**:
1. Understand ROS 2 as middleware for robot control
2. Create and run ROS 2 nodes in Python
3. Implement communication via topics and services
4. Define robot geometry using URDF

**Chapter Outline**:
1. **ROS 2 Fundamentals** (12 min): What is ROS 2, why it matters, distributed systems concept
2. **Your First ROS 2 Node** (12 min): Hands-on tutorial to create, build, run a simple publisher
3. **Topics and Services** (12 min): Communication patterns, when to use each, examples
4. **Python and rclpy** (12 min): Bridging AI agents (Python) to robot controllers
5. **URDF for Humanoids** (12 min): Robot description format, joints, links, visual vs collision meshes

**Prerequisites**: Basic Python, command line comfort, virtual environments
**Hands-on Exercises**: 3 tutorials (first node, topic publisher/subscriber, URDF visualization)

### Module 2: The Digital Twin (Gazebo & Unity)

**Learning Objectives**:
1. Simulate physics in Gazebo (gravity, collisions, friction)
2. Render high-fidelity environments in Unity
3. Configure virtual sensors (LiDAR, depth cameras, IMUs)
4. Connect simulation to ROS 2 for testing

**Chapter Outline**:
1. **Gazebo Introduction** (12 min): What is physics simulation, why simulate before deploying
2. **Physics Simulation** (12 min): Gravity, rigid body dynamics, collision detection
3. **Unity for HRI** (12 min): High-fidelity rendering, human-robot interaction scenarios
4. **Sensor Simulation** (12 min): Configuring LiDAR, depth cameras (RealSense), IMUs in Gazebo
5. **Gazebo-ROS 2 Bridge** (12 min): Connecting simulated robots to ROS 2 control nodes

**Prerequisites**: Module 1 completed (ROS 2 basics)
**Hands-on Exercises**: 2 tutorials (Gazebo world setup, sensor data visualization)

### Module 3: The AI-Robot Brain (NVIDIA Isaac)

**Learning Objectives**:
1. Use Isaac Sim for photorealistic robot simulation
2. Generate synthetic training data for perception models
3. Implement hardware-accelerated VSLAM with Isaac ROS
4. Plan paths for bipedal humanoid navigation

**Chapter Outline**:
1. **Isaac Sim Overview** (12 min): What is Isaac Sim, why photorealism matters, USD format
2. **Isaac Sim Setup** (12 min): Installation, GPU requirements (RTX), first scene
3. **Synthetic Data Generation** (12 min): Creating training datasets for vision models
4. **Isaac ROS Perception** (12 min): Hardware-accelerated perception pipelines on Jetson
5. **Visual SLAM** (12 min): Simultaneous localization and mapping with cameras
6. **Nav2 for Bipedal Robots** (12 min): Path planning challenges for humanoid balance and gait

**Prerequisites**: Modules 1-2 completed (ROS 2 + simulation basics)
**Hands-on Exercises**: 2 tutorials (Isaac Sim scene, Isaac ROS VSLAM demo)

### Module 4: Vision-Language-Action (VLA)

**Learning Objectives**:
1. Understand the convergence of LLMs and robotics
2. Implement voice-to-action pipelines with Whisper
3. Use LLMs for high-level task planning
4. Integrate VLA systems with ROS 2 robots

**Chapter Outline**:
1. **VLA Overview** (12 min): What is Vision-Language-Action, the embodied AI paradigm
2. **Voice-to-Action** (12 min): OpenAI Whisper for speech recognition, command parsing
3. **LLM Cognitive Planning** (12 min): Translating natural language to robot actions
4. **VLA-ROS 2 Integration** (12 min): Connecting LLM planners to ROS 2 control loops
5. **Case Studies** (12 min): Real-world VLA applications (household robots, warehouse automation)

**Prerequisites**: Modules 1-3 completed (ROS 2 + simulation + perception)
**Hands-on Exercises**: 2 tutorials (Whisper voice command, LLM planning demo)

### Supporting Content

**Introduction (15 min)**:
- What is Physical AI and Embodied Intelligence?
- Why this matters: AI leaving the digital realm
- Learning journey overview and prerequisites

**Quick Start (15 min)**:
- Fastest path to running first ROS 2 node
- "Hello World" of robotics simulation
- What you'll build: simple voice-controlled simulated robot

**Hardware Guidance (30 min)**:
- Workstation requirements: RTX GPU, CPU, RAM, Ubuntu 22.04
- Edge compute kit: Jetson Orin, RealSense camera, IMU
- Robot lab options: 3 budget tiers with cost breakdowns

**Glossary (10 min)**:
- Key terms: ROS 2, URDF, Gazebo, Isaac Sim, VSLAM, VLA, rclpy, Nav2, USD, etc.
- Cross-referenced to chapters where terms are introduced

## Content Creation Standards

### Chapter Structure Template

Each chapter follows consistent format:
```markdown
---
id: unique-chapter-id
title: Chapter Title
sidebar_position: N
---

## Learning Objectives
- Objective 1 (measurable)
- Objective 2 (measurable)
- Objective 3 (measurable)

## Prerequisites
- Prior knowledge required
- Previous chapters to complete

## Introduction
[2-3 paragraphs: why this topic matters, what you'll build]

## Core Concepts
### Concept 1
[Explanation with diagrams]

### Concept 2
[Explanation with code examples]

## Hands-On Tutorial
[Step-by-step walkthrough with code blocks]

```python
# Code example with comments
import rclpy
# ... runnable example
```

## Summary
[Key takeaways, what you learned]

## Next Steps
[Preview of next chapter, additional resources]
```

### Code Example Standards

All code examples must:
- Be **runnable** (copy-paste should work)
- Include **inline comments** explaining each section
- Use **consistent formatting** (Black for Python, 4-space indent)
- Show **expected output** in comments or text
- Reference **specific versions** (e.g., `rclpy` for ROS 2 Humble)

Example:
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
        rclpy.spin(node)  # Keep node running
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

### Visual Diagram Standards

All diagrams must:
- Be **SVG format** (scalable, small file size)
- Include **descriptive alt text** for accessibility
- Use **consistent color scheme** (define palette in research.md)
- Show **data flow** or **system architecture** clearly
- Be **referenced in text** with figure numbers

Required diagrams (8 total):
1. **ROS 2 Node Communication**: Publisher-subscriber pattern
2. **ROS 2 Service Pattern**: Request-response flow
3. **Gazebo Physics Pipeline**: Simulation loop (physics → rendering → sensors)
4. **URDF Robot Structure**: Links, joints, coordinate frames
5. **Isaac Sim Workflow**: Scene → USD → Rendering → Synthetic Data
6. **VSLAM Pipeline**: Camera → Feature Extraction → Mapping → Localization
7. **VLA Architecture**: Voice → LLM → Action Plan → Robot Execution
8. **Sim-to-Real Transfer**: Simulation training → Deployment on physical robot

## Technology Decisions

### Docusaurus Configuration

**Version**: Docusaurus 3.x (latest stable)
**Theme**: Classic theme with custom color scheme
**Plugins**:
- `@docusaurus/plugin-content-docs` (core documentation)
- `@docusaurus/plugin-content-pages` (custom pages)
- `@docusaurus/preset-classic` (batteries-included preset)
- `docusaurus-plugin-search-local` (offline search, fallback if Algolia unavailable)

**Color Palette** (Physical AI theme):
- Primary: `#2E3440` (dark blue-gray, representing "physical")
- Secondary: `#88C0D0` (light blue, representing "AI")
- Accent: `#A3BE8C` (green, representing "robotics/life")
- Code background: `#ECEFF4` (light gray)
- Syntax highlighting: Prism Night Owl theme

### GitHub Pages Deployment

**Strategy**: GitHub Actions workflow triggers on push to `main` branch
**Build Command**: `npm run build` (outputs to `build/` directory)
**Deployment**: GitHub Actions deploys `build/` directory to `gh-pages` branch
**Custom Domain**: Optional (supports custom domain if user configures DNS)

**GitHub Actions Workflow** (`.github/workflows/deploy.yml`):
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
      - run: npm ci
      - run: npm run build
      - uses: actions/upload-pages-artifact@v2
        with:
          path: build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/deploy-pages@v2
        id: deployment
```

### Search Implementation

**Primary**: Algolia DocSearch (free for open source documentation)
**Fallback**: `docusaurus-plugin-search-local` (offline search if Algolia not available)
**Index**: All markdown content, headings, and code block titles

### Responsive Design

**Breakpoints**:
- Mobile: < 768px (single column, collapsible sidebar)
- Tablet: 768px - 1024px (sidebar visible, optimized layout)
- Desktop: > 1024px (full sidebar, code examples side-by-side)

**Mobile Optimizations**:
- Collapsible code blocks with "Show code" toggle
- Touch-friendly navigation (larger tap targets)
- Lazy-load images for faster mobile load times

## Research Summary

**Content Structure**: 4 modules with 5-6 chapters each, totaling ~40 pages of content, 4-6 hour reading time
**Technology Stack**: Docusaurus 3.x, React 18, MDX 3, Node.js 18+, GitHub Pages deployment
**Code Standards**: Runnable examples, inline comments, Python 3.10+, ROS 2 Humble/Iron
**Visual Standards**: SVG diagrams, consistent color palette, alt text for accessibility
**Performance**: Static site generation, CDN delivery via GitHub Pages, < 3 second load time
**Deployment**: Automated GitHub Actions workflow, builds on every push to main

**Risks Identified**:
1. **Content scope creep**: Mitigate by strict 12-minute chapter time budget
2. **Code examples breaking with version updates**: Mitigate by explicit version pinning (ROS 2 Humble, Isaac Sim 2023.1.x)
3. **Diagram creation time**: Mitigate by using draw.io templates and simple architectural diagrams

**Next Phase**: Generate data-model.md to formalize content entities (Module, Chapter, Code Example) and their metadata schemas.
