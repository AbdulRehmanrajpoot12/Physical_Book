import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Physical_Book/blog',
    component: ComponentCreator('/Physical_Book/blog', '7ee'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/archive',
    component: ComponentCreator('/Physical_Book/blog/archive', '0b3'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/authors',
    component: ComponentCreator('/Physical_Book/blog/authors', 'f64'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/Physical_Book/blog/authors/all-sebastien-lorber-articles', '77b'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/authors/yangshun',
    component: ComponentCreator('/Physical_Book/blog/authors/yangshun', 'b55'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/first-blog-post',
    component: ComponentCreator('/Physical_Book/blog/first-blog-post', 'c72'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/long-blog-post',
    component: ComponentCreator('/Physical_Book/blog/long-blog-post', '658'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/mdx-blog-post',
    component: ComponentCreator('/Physical_Book/blog/mdx-blog-post', '714'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/tags',
    component: ComponentCreator('/Physical_Book/blog/tags', '42f'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/tags/docusaurus',
    component: ComponentCreator('/Physical_Book/blog/tags/docusaurus', 'bc6'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/tags/facebook',
    component: ComponentCreator('/Physical_Book/blog/tags/facebook', '501'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/tags/hello',
    component: ComponentCreator('/Physical_Book/blog/tags/hello', '250'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/tags/hola',
    component: ComponentCreator('/Physical_Book/blog/tags/hola', '450'),
    exact: true
  },
  {
    path: '/Physical_Book/blog/welcome',
    component: ComponentCreator('/Physical_Book/blog/welcome', '4e2'),
    exact: true
  },
  {
    path: '/Physical_Book/markdown-page',
    component: ComponentCreator('/Physical_Book/markdown-page', 'd5e'),
    exact: true
  },
  {
    path: '/Physical_Book/docs',
    component: ComponentCreator('/Physical_Book/docs', '445'),
    routes: [
      {
        path: '/Physical_Book/docs',
        component: ComponentCreator('/Physical_Book/docs', '2f5'),
        routes: [
          {
            path: '/Physical_Book/docs',
            component: ComponentCreator('/Physical_Book/docs', '79b'),
            routes: [
              {
                path: '/Physical_Book/docs/',
                component: ComponentCreator('/Physical_Book/docs/', 'b40'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/intro',
                component: ComponentCreator('/Physical_Book/docs/intro', 'b2b'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/module-1/python-agents-rclpy',
                component: ComponentCreator('/Physical_Book/docs/module-1/python-agents-rclpy', '65a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-1/ros2-basics',
                component: ComponentCreator('/Physical_Book/docs/module-1/ros2-basics', 'd19'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-1/urdf-humanoids',
                component: ComponentCreator('/Physical_Book/docs/module-1/urdf-humanoids', 'c95'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-2/digital-twins-physical-ai',
                component: ComponentCreator('/Physical_Book/docs/module-2/digital-twins-physical-ai', '8b4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-2/gazebo-physics-simulation',
                component: ComponentCreator('/Physical_Book/docs/module-2/gazebo-physics-simulation', '8e6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-2/unity-high-fidelity-interaction',
                component: ComponentCreator('/Physical_Book/docs/module-2/unity-high-fidelity-interaction', 'f95'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-3/',
                component: ComponentCreator('/Physical_Book/docs/module-3/', 'b28'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/module-3/isaac-ros-vslam',
                component: ComponentCreator('/Physical_Book/docs/module-3/isaac-ros-vslam', 'f79'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-3/nav2-humanoid-navigation',
                component: ComponentCreator('/Physical_Book/docs/module-3/nav2-humanoid-navigation', '90b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-3/nvidia-isaac-sim',
                component: ComponentCreator('/Physical_Book/docs/module-3/nvidia-isaac-sim', '540'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-4/',
                component: ComponentCreator('/Physical_Book/docs/module-4/', '645'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-4/autonomous-humanoid',
                component: ComponentCreator('/Physical_Book/docs/module-4/autonomous-humanoid', 'bba'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-4/llm-planning',
                component: ComponentCreator('/Physical_Book/docs/module-4/llm-planning', 'b31'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/module-4/voice-to-action',
                component: ComponentCreator('/Physical_Book/docs/module-4/voice-to-action', '647'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical_Book/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/Physical_Book/docs/tutorial-basics/congratulations', 'a93'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/Physical_Book/docs/tutorial-basics/create-a-blog-post', '2be'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/Physical_Book/docs/tutorial-basics/create-a-document', '84b'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/Physical_Book/docs/tutorial-basics/create-a-page', '13e'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/Physical_Book/docs/tutorial-basics/deploy-your-site', '5ed'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/Physical_Book/docs/tutorial-basics/markdown-features', '8af'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/Physical_Book/docs/tutorial-extras/manage-docs-versions', '932'),
                exact: true
              },
              {
                path: '/Physical_Book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/Physical_Book/docs/tutorial-extras/translate-your-site', '8d4'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/Physical_Book/',
    component: ComponentCreator('/Physical_Book/', '84b'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
