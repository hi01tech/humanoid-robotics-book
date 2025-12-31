// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Textbook',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://hi01tech.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  // baseUrl: '/humanoid-robotics-book/',
  baseUrl: process.env.VERCEL ? '/' : '/humanoid-robotics-book/',
  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'hi01tech', // Usually your GitHub org/user name.
  projectName: 'humanoid-robotics-book', // Usually your repo name.

  deploymentBranch: 'gh-pages',
  trailingSlash: false, // Helps Vercel and GitHub agree on links


  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
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
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Logo',
          src: 'img/logo.png',
          width: 50,
          height: 50,
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {to: '/search', label: 'Search', position: 'right'},
          {to: '/chatbot', label: 'Chatbot', position: 'right'},
        ],
      },
      footer: {
        style: 'dark',
        links: [],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),
};

module.exports = config;
