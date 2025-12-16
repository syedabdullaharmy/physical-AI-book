import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master Embodied Intelligence and Robot Control Systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://yourusername.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'yourusername', // Your GitHub org/user name.
  projectName: 'book', // Your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // internationalization support (future: Urdu)
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: 'docs',
          editUrl: 'https://github.com/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    async function tailwindPlugin(context, options) {
      return {
        name: 'docusaurus-tailwindcss',
        configurePostCss(postcssOptions) {
          postcssOptions.plugins.push(require('tailwindcss'));
          postcssOptions.plugins.push(require('autoprefixer'));
          return postcssOptions;
        },
      };
    },
  ],

  themeConfig: {
    // Social card for sharing
    image: 'img/social-card.png',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Course',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'courseSidebar',
          position: 'left',
          label: 'Course',
        },
        {
          to: '/docs/hardware',
          label: 'Hardware',
          position: 'left',
        },
        {
          to: '/docs/assessments',
          label: 'Assessments',
          position: 'left',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          to: '/login',
          label: 'Login',
          position: 'right',
        },
        {
          to: '/register',
          label: 'Register',
          position: 'right',
        },

        {
          href: 'https://github.com',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1/chapter-1',
            },
            {
              label: 'Module 2: Gazebo & Unity',
              to: '/docs/module-2/chapter-6',
            },
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-3/chapter-10',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/module-4/chapter-14',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Hardware Requirements',
              to: '/docs/hardware',
            },
            {
              label: 'Lab Setup',
              to: '/docs/lab-setup',
            },
            {
              label: 'Assessments',
              to: '/docs/assessments',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com',
            },
            {
              label: 'Issues',
              href: 'https://github.com',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.vsDark,
      additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
