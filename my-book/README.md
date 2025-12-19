# Physical AI & Humanoid Robotics Book

This is a technical book on ROS 2 for Physical AI & Humanoid Robotics. The content is organized in modules with each module focusing on specific aspects of ROS 2 and its application to humanoid robotics.

## Module 1: The Robotic Nervous System (ROS 2)

This module covers:

1. **ROS 2 Basics** - Understanding middleware in robotics, ROS 2 vs ROS 1, distributed systems, and architecture overview
2. **Python Agents & rclpy** - ROS 2 communication patterns (nodes, topics, services) and humanoid data flow examples
3. **URDF for Humanoids** - rclpy and Python AI agents integration, URDF fundamentals, and humanoid structure (links and joints)

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
