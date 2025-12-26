# Quickstart Guide: Digital Twin Simulation for Physical AI & Humanoid Robotics

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git
- Students should have completed Module 1 (ROS 2 fundamentals)
- Basic understanding of physics concepts (gravity, collisions, forces)

## Adding Module 2 Content

1. **Create the module directory:**
   ```bash
   mkdir -p frontend_book/my-book/docs/module-2
   ```

2. **Create the three required chapters:**
   ```bash
   touch frontend_book/my-book/docs/module-2/digital-twins-physical-ai.md
   touch frontend_book/my-book/docs/module-2/gazebo-physics-simulation.md
   touch frontend_book/my-book/docs/module-2/unity-high-fidelity-interaction.md
   ```

3. **Add the chapters to the sidebar configuration** in `frontend_book/my-book/sidebars.js`:
   ```javascript
   // frontend_book/my-book/sidebars.js
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
           'module-2/digital-twins-physical-ai',
           'module-2/gazebo-physics-simulation',
           'module-2/unity-high-fidelity-interaction',
         ],
       },
     ],
   };
   ```

## Development Workflow

1. Edit the `.md` files in the `frontend_book/my-book/docs/module-2/` directory
2. Preview changes automatically with `npm start` (from the frontend_book/my-book directory)
3. When ready, build with `npm run build` and deploy to GitHub Pages
4. Commit your changes to version control

## Gazebo Setup for Students

For students to follow along with Gazebo examples:
- Install ROS 2 Humble Hawksbill or Iron Irwini
- Install Gazebo Harmonic or Garden (matching ROS 2 version)
- Set up a basic robot model (URDF from Module 1 can be reused)

## Unity Setup for Students

For students to follow along with Unity examples:
- Install Unity Hub and Unity 2022.3 LTS
- Install the ROS# package for Unity-ROS communication
- Set up basic scene with humanoid robot model

## Next Steps

- Write content for each chapter following the educational objectives
- Add code examples and configuration files as needed
- Test the deployment process to GitHub Pages
- Verify all links and navigation work correctly