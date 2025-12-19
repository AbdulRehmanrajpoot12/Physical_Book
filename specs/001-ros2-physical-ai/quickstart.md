# Quickstart Guide: ROS 2 for Physical AI & Humanoid Robotics

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git
- Basic Python knowledge (Python 3.8+)

## Installation

1. **Create a new Docusaurus project:**
   ```bash
   npx create-docusaurus@latest my-book classic
   ```

2. **Navigate to your project directory:**
   ```bash
   cd my-book
   ```

3. **Start the development server:**
   ```bash
   npm start
   ```

4. **Create the module directory:**
   ```bash
   mkdir -p docs/module-1
   ```

## Adding Module Content

1. **Create the three required chapters:**
   ```bash
   touch docs/module-1/ros2-basics.md
   touch docs/module-1/python-agents-rclpy.md
   touch docs/module-1/urdf-humanoids.md
   ```

2. **Add the chapters to the sidebar configuration** in `sidebars.js`:
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
     ],
   };
   ```

## Building for GitHub Pages

1. **Build the static site:**
   ```bash
   npm run build
   ```

2. **Test locally before deployment:**
   ```bash
   npm run serve
   ```

3. **Configure GitHub Pages deployment** in your GitHub repository settings to use the `gh-pages` branch.

## Development Workflow

1. Edit the `.md` files in the `docs/module-1/` directory
2. Preview changes automatically with `npm start`
3. When ready, build with `npm run build` and deploy to GitHub Pages
4. Commit your changes to version control

## Next Steps

- Write content for each chapter following the educational objectives
- Add code examples and diagrams as needed
- Test the deployment process to GitHub Pages
- Verify all links and navigation work correctly