# Quickstart Guide: Module 4 - Vision-Language-Action (VLA)

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git
- Students should have completed Modules 1, 2, and 3 (ROS 2, simulation, and NVIDIA Isaac fundamentals)
- Basic understanding of LLMs and speech recognition concepts

## Adding Module 4 Content

1. **Create the module directory:**
   ```bash
   mkdir -p my-book/docs/module-4
   ```

2. **Create the three required chapters:**
   ```bash
   touch my-book/docs/module-4/voice-to-action.md
   touch my-book/docs/module-4/llm-planning.md
   touch my-book/docs/module-4/autonomous-humanoid.md
   ```

3. **Add the chapters to the sidebar configuration** in `sidebars.js`:
   ```javascript
   module.exports = {
     tutorialSidebar: [
       {
         type: 'category',
         label: 'Module 1: The Robotic Nervous System (ROS 2)',
         items: [
           'module-1/ros2-basics',
           'module-1/python-agents-rclpy',
           'module-1/urdf-humanoids',
         ],
       },
       {
         type: 'category',
         label: 'Module 2: The Digital Twin (Gazebo & Unity)',
         items: [
           'module-2/index',
           'module-2/digital-twins-physical-ai',
           'module-2/gazebo-physics-simulation',
           'module-2/unity-high-fidelity-interaction',
         ],
       },
       {
         type: 'category',
         label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
         items: [
           'module-3/index',
           'module-3/nvidia-isaac-sim',
           'module-3/isaac-ros-vslam',
           'module-3/nav2-humanoid-navigation',
         ],
       },
       {
         type: 'category',
         label: 'Module 4: Vision-Language-Action (VLA)',
         items: [
           'module-4/voice-to-action',
           'module-4/llm-planning',
           'module-4/autonomous-humanoid',
         ],
       },
     ],
   };
   ```

## Development Workflow

1. Edit the `.md` files in the `docs/module-4/` directory
2. Preview changes automatically with `npm start` (from the my-book directory)
3. When ready, build with `npm run build` and deploy to GitHub Pages
4. Commit your changes to version control

## Vision-Language-Action Concepts for Students

For students to understand the concepts covered in this module:
- Review OpenAI Whisper documentation for speech recognition concepts
- Study LLM capabilities for natural language understanding and planning
- Understand ROS 2 action interfaces for robot execution
- Learn about voice command intent classification techniques

## Next Steps

- Write content for each chapter following the educational objectives
- Add conceptual examples and diagrams as needed
- Test the deployment process to GitHub Pages
- Verify all links and navigation work correctly