# Quickstart Guide: The AI-Robot Brain (NVIDIA Isaac™)

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git
- Students should have completed Modules 1 and 2 (ROS 2 and simulation fundamentals)
- Basic understanding of perception and navigation concepts

## Adding Module 3 Content

1. **Create the module directory:**
   ```bash
   mkdir -p frontend_book/my-book/docs/module-3
   ```

2. **Create the three required chapters:**
   ```bash
   touch frontend_book/my-book/docs/module-3/nvidia-isaac-sim.md
   touch frontend_book/my-book/docs/module-3/isaac-ros-vslam.md
   touch frontend_book/my-book/docs/module-3/nav2-humanoid-navigation.md
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
           'module-3/nvidia-isaac-sim',
           'module-3/isaac-ros-vslam',
           'module-3/nav2-humanoid-navigation',
         ],
       },
     ],
   };
   ```

## Development Workflow

1. Edit the `.md` files in the `frontend_book/my-book/docs/module-3/` directory
2. Preview changes automatically with `npm start` (from the frontend_book/my-book directory)
3. When ready, build with `npm run build` and deploy to GitHub Pages
4. Commit your changes to version control

## NVIDIA Isaac Concepts for Students

For students to understand the concepts covered in this module:
- Review Isaac Sim documentation for simulation concepts
- Study Isaac ROS for perception integration
- Understand Navigation2 (Nav2) for mobile robot navigation
- Learn about Visual SLAM (VSLAM) for localization and mapping

## Next Steps

- Write content for each chapter following the educational objectives
- Add conceptual examples and diagrams as needed
- Test the deployment process to GitHub Pages
- Verify all links and navigation work correctly