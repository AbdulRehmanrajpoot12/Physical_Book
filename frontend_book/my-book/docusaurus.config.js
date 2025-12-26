// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).

import { themes as prismThemes } from "prism-react-renderer";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "The Robotic Nervous System (ROS 2)",
  favicon: "img/favicon.ico",

  // Future flags
  future: {
    v4: true,
  },

  // Production URL & base URL for GitHub Pages
  url: "https://AbdulRehmanrajpoot12.github.io",
  baseUrl: "/Physical_Book/",

  // GitHub pages deployment config
  organizationName: "AbdulRehmanrajpoot12",
  projectName: "Physical_Book",

  onBrokenLinks: "throw",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: "./sidebars.js",
          editUrl:
            "https://github.com/AbdulRehmanrajpoot12/Physical_Book/edit/main/",
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ["rss", "atom"],
            xslt: true,
          },
          editUrl:
            "https://github.com/AbdulRehmanrajpoot12/Physical_Book/edit/main/",
          onInlineTags: "warn",
          onInlineAuthors: "warn",
          onUntruncatedBlogPosts: "warn",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      }),
    ],
  ],

  plugins: [],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: "img/docusaurus-social-card.jpg",
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: "Physical AI & Humanoid Robotics",
        logo: {
          alt: "Physical AI & Humanoid Robotics Logo",
          src: "img/logo.svg",
        },
        items: [
          {
            type: "docSidebar",
            sidebarId: "tutorialSidebar",
            position: "left",
            label: "Module 1: ROS 2",
          },
          {
            href: "https://github.com/AbdulRehmanrajpoot12/Physical_Book",
            label: "GitHub",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        links: [
          {
            title: "Docs",
            items: [
              {
                label: "Module 1: ROS 2",
                to: "/docs/module-1/ros2-basics",
              },
            ],
          },
          {
            title: "Community",
            items: [
              {
                label: "ROS Answers",
                href: "https://answers.ros.org/",
              },
              {
                label: "Discord",
                href: "https://discord.gg/robotics",
              },
            ],
          },
          {
            title: "More",
            items: [
              {
                label: "GitHub",
                href: "https://github.com/AbdulRehmanrajpoot12/Physical_Book",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
