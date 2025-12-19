# Physical AI & Humanoid Robotics Book

This is a technical book on ROS 2 and Digital Twin Simulation for Physical AI & Humanoid Robotics. The content is organized in modules with each module focusing on specific aspects of robotics and AI integration.

## Module 1: The Robotic Nervous System (ROS 2)

This module covers:

1. **ROS 2 Basics** - Understanding middleware in robotics, ROS 2 vs ROS 1, distributed systems, and architecture overview
2. **Python Agents & rclpy** - ROS 2 communication patterns (nodes, topics, services) and humanoid data flow examples
3. **URDF for Humanoids** - rclpy and Python AI agents integration, URDF fundamentals, and humanoid structure (links and joints)

## Module 2: The Digital Twin (Gazebo & Unity)

This module covers:

1. **Digital Twins for Physical AI** - Definition of digital twins, role of simulation in embodied intelligence, and why humanoid robots require simulated environments
2. **Physics Simulation with Gazebo** - Simulating physics, gravity, and collisions, humanoid robot interaction with environments, and sensor simulation (LiDAR, depth cameras, IMUs)
3. **High-Fidelity Simulation with Unity** - Visual realism and human-robot interaction, Unity's role alongside Gazebo, and simulation for training, testing, and validation

## Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This module covers:

1. **NVIDIA Isaac Sim** - Photorealistic simulation, synthetic data generation, and training perception models
2. **Isaac ROS & VSLAM** - Hardware-accelerated perception, Visual SLAM (VSLAM), and navigation foundations
3. **Nav2 for Humanoid Navigation** - Path planning concepts, navigation for bipedal humanoids, and integration with perception systems

## Module 4: Vision-Language-Action (VLA)

This module covers:

1. **Voice-to-Action** - Speech input using OpenAI Whisper and converting voice commands to robot intents
2. **Cognitive Planning with LLMs** - Translating natural language into action sequences and mapping plans to ROS 2 actions
3. **Capstone – The Autonomous Humanoid** - End-to-end system overview with perception, navigation, manipulation pipeline

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using GitHub Actions (recommended):

The site is configured for GitHub Pages deployment. After pushing to the main branch, the site will be automatically deployed to GitHub Pages.

Manual deployment:

```bash
npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
