/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

// @ts-check

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Book',
  tagline: 'Bridging Digital Minds to Physical Bodies',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Physical-AI-Humanoid-Robotics/',

  // GitHub pages deployment config.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Enable i18n for Urdu language support
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      ur: {
        label: 'اردو',
        direction: 'rtl', // Right to left for Urdu
      },
      en: {
        label: 'English',
        direction: 'ltr',
      }
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      docs: {
        sidebar: {
          hideable: true,
        },
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      // Disable the back-to-top button to fix SSR error
      announcementBar: {
        id: 'announcementBar-1', // ID of the announcement bar
        content: 'Welcome to the Physical AI & Humanoid Robotics Textbook!',
        isCloseable: true,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
  { to: '/', label: 'Home', position: 'left' },

  // Language dropdown
  { type: 'localeDropdown', position: 'right' },

  {
    type: 'html',
    position: 'right',
    value: '<div id="auth-navbar-placeholder"></div>',
  },

  // GitHub link
  {
    href: 'https://github.com/syedsaudali-gh/Physical-AI-Humanoid-Robotics',
    className: 'header-github-link',
    'aria-label': 'GitHub repository',
    position: 'right',
  },
],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Book Introduction',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/syed-saud-ali-6399712b4/',
              },
              {
                label: 'Facebook',
                href: 'https://www.facebook.com/saud.saleem.391',
              },
              {
                label: 'X (Twitter)',
                href: 'https://x.com/saudali242821',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/syedsaudali-gh/Physical-AI-Humanoid-Robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),

  // Development server proxy configuration
  staticDirectories: ['static'],

  // Proxy /api to backend during development
  themes: [
    // Add theme plugins here
  ],

  // Client modules for custom functionality
  clientModules: [],

  plugins: [
    // Google Analytics for performance monitoring
    [
      '@docusaurus/plugin-google-gtag',
      {
        trackingID: 'G-XXXXXXXXXX', // Replace with actual tracking ID in production
        anonymizeIP: true,
      },
    ],
  ],


  // Place non-standard custom fields in customFields
  customFields: {
    devServer: {
      proxy: {
        '/api': {
          target: 'http://localhost:8000/', // Backend server address
          changeOrigin: true,
          secure: false, // Set to true if backend uses HTTPS
          logLevel: 'debug',
          onError: (err, req, res) => {
            console.error('Proxy error:', err);
            res.writeHead(500, {
              'Content-Type': 'application/json',
            });
            res.end(JSON.stringify({ error: 'Proxy error occurred' }));
          },
        },
      },
    },
  },
};

module.exports = config;