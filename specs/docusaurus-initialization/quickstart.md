# Quickstart: Setting Up the AI-Physical Book

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git
- Text editor

## Installation Steps

### 1. Initialize Docusaurus Project

```bash
npx create-docusaurus@latest frontend_book/my-book classic
cd frontend_book/my-book
```

### 2. Project Structure Overview

After initialization, your project will have the following structure:

```
frontend_book/my-book/
├── blog/
├── docs/
├── src/
│   ├── components/
│   ├── css/
│   └── pages/
├── static/
├── docusaurus.config.js
├── package.json
├── sidebars.js
└── README.md
```

### 3. Start Local Development Server

```bash
npm start
```

This command starts a local development server and opens your site in a browser at `http://localhost:3000`.

### 4. Create Module 1 Structure

Create the directory for Module 1 and the three chapters:

```bash
mkdir frontend_book/my-book/docs/module-1
touch frontend_book/my-book/docs/module-1/ros2-basics.md
touch frontend_book/my-book/docs/module-1/python-agents-rclpy.md
touch frontend_book/my-book/docs/module-1/urdf.md
```

### 5. Configure Sidebar Navigation

Update `frontend_book/my-book/sidebars.js` to include the new modules and chapters:

```javascript
// frontend_book/my-book/sidebars.js
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1/ros2-basics',
        'module-1/python-agents-rclpy',
        'module-1/urdf',
      ],
    },
  ],
};
```

### 6. Add Content to Chapters

Each chapter should follow this basic structure:

```markdown
---
title: Chapter Title
description: Brief description of the chapter content
---

# Chapter Title

## Learning Objectives

- Objective 1
- Objective 2
- Objective 3

## Introduction

Introductory content...

## Main Content

### Section 1

Content for section 1...

### Section 2

Content for section 2...

## Summary

Summary of key points covered in the chapter.

## Exercises

Optional exercises for readers to practice.
```

### 7. Build for Production

```bash
npm run build
```

This creates an optimized production build in the `build/` directory.

### 8. Deploy to GitHub Pages

1. Create a GitHub repository for your book
2. Push your Docusaurus project to the repository
3. Configure GitHub Pages in repository settings to use the `gh-pages` branch
4. Run the deployment command:

```bash
GIT_USER=<Your GitHub username> \
  CURRENT_BRANCH=main \
  USE_SSH=true \
  npm run deploy
```

## Configuration Options

### docusaurus.config.js

Key configuration options for your book:

- `title`: The name of your website
- `tagline`: A tagline for your website
- `url`: URL of your website
- `baseUrl`: Base URL for your project
- `organizationName`: GitHub account/organization name
- `projectName`: GitHub repository name
- `deploymentBranch`: Branch for deploying GitHub pages
- `favicon`: Path to your site favicon

### Styling

Customize the look and feel by modifying:
- `src/css/custom.css` for custom styles
- `src/pages/` for custom pages
- `static/` for static assets like images

## Next Steps

1. Populate the three Module 1 chapters with content
2. Add additional modules as needed
3. Customize the site's appearance and navigation
4. Set up automated deployment to GitHub Pages