// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const {themes} = require('prism-react-renderer');
const nightOwlTheme = themes.nightOwl;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A hands-on guide to building intelligent robots with ROS 2, Isaac Sim, and Vision-Language-Action models',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // Custom fields - API configuration
  customFields: {
    // API URL - defaults to localhost for dev, set DOCUSAURUS_API_URL env var for production
    apiUrl: process.env.DOCUSAURUS_API_URL || 'http://localhost:8000',
  },

  // GitHub pages deployment config
  organizationName: 'username', // Usually your GitHub org/user name
  projectName: 'Physical-ai-human-robotics-book', // Usually your repo name

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Internationalization (optional, can be configured later)
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: '/docs', // Docs at /docs, allowing custom homepage at /
          sidebarPath: require.resolve('./sidebars.js'),
          // Remove this to remove the "edit this page" links
          editUrl:
            'https://github.com/username/Physical-ai-human-robotics-book/tree/main/',
        },
        blog: false, // Disable blog for this book
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themes: [
    [
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            to: '/ai-chatbot',
            position: 'left',
            label: 'AI Chatbot',
          },
          {
            href: 'https://github.com/username/Physical-ai-human-robotics-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book',
            items: [
              {
                label: 'Home',
                to: '/',
              },
              {
                label: 'Quick Start',
                to: '/docs/quick-start',
              },
              {
                label: 'Hardware Setup',
                to: '/docs/hardware',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/username/Physical-ai-human-robotics-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Education Team. Built with Docusaurus.`,
      },
      prism: {
        theme: nightOwlTheme,
        darkTheme: nightOwlTheme,
        additionalLanguages: ['bash', 'python', 'yaml', 'json'],
      },
      // Physical AI theme colors
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),
};

module.exports = config;
