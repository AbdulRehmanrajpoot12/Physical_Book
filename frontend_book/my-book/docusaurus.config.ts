import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "The Robotic Nervous System (ROS 2)",
  favicon: "img/favicon.ico",

  // Future flags
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Production URL & base URL for GitHub Pages
  url: "https://AbdulRehmanrajpoot12.github.io",
  baseUrl: "/ai-physical-book/",

  // GitHub pages deployment config
  organizationName: "AbdulRehmanrajpoot12", // Your GitHub username
  projectName: "ai-physical-book", // Your repo name

  onBrokenLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.js",
          editUrl:
            "https://github.com/AbdulRehmanrajpoot12/ai-physical-book/edit/main/",
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ["rss", "atom"],
            xslt: true,
          },
          editUrl:
            "https://github.com/AbdulRehmanrajpoot12/ai-physical-book/edit/main/",
          onInlineTags: "warn",
          onInlineAuthors: "warn",
          onUntruncatedBlogPosts: "warn",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [],
  clientModules: ["./src/plugins/ChatbotPlugin.js"],

  themeConfig: {
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
          href: "https://github.com/AbdulRehmanrajpoot12/ai-physical-book",
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
              href: "https://github.com/AbdulRehmanrajpoot12/ai-physical-book",
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
  } satisfies Preset.ThemeConfig,
};

export default config;
