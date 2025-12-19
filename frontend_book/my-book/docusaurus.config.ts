import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const config: Config = {
  title: "ROS 2 for Physical AI & Humanoid Robotics",
  tagline:
    "A comprehensive guide to ROS 2 fundamentals, Python agents, and URDF for humanoid robots",
  favicon: "img/favicon.ico",

  future: {
    v4: true,
  },

  url: "https://AbdulRehmanrajpoot12.github.io",
  baseUrl: "/Physical_Book/",
  trailingSlash: true, // ✅ important for GitHub Pages

  organizationName: "AbdulRehmanrajpoot12",
  projectName: "Physical_Book",

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
          sidebarPath: "./sidebars.ts",
          editUrl:
            "https://github.com/AbdulRehmanrajpoot12/Physical_Book/tree/main/",
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ["rss", "atom"],
            xslt: true,
          },
          editUrl:
            "https://github.com/AbdulRehmanrajpoot12/Physical_Book/tree/main/",
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

  themeConfig: {
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "ROS 2 for Physical AI",
      logo: {
        alt: "ROS 2 Logo",
        src: "img/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Module 1",
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
              label: "Module 1: ROS 2 Fundamentals",
              to: "/docs/module-1/ros2-basics",
            },
          ],
        },
        {
          title: "ROS 2 Resources",
          items: [
            {
              label: "Official ROS 2 Documentation",
              href: "https://docs.ros.org/en/humble/",
            },
            {
              label: "ROS 2 Tutorials",
              href: "https://docs.ros.org/en/humble/Tutorials.html",
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
      copyright: `Copyright © ${new Date().getFullYear()} ROS 2 for Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
