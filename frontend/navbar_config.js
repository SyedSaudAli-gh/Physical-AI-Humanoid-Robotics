// Complete navbar configuration for Docusaurus site
// This is the themeConfig.navbar object for docusaurus.config.js

/**
 * Full navbar configuration for Physical AI & Humanoid Robotics Docusaurus site
 * 
 * To use this configuration:
 * 1. Add this to your docusaurus.config.js file in the themeConfig.navbar property
 * 2. Ensure i18n is set up with 'en' locale in your docusaurus.config.js
 * 3. Create /login and /signup pages in your src/pages directory
 */
const navbarConfig = {
  title: 'Physical AI & Humanoid Robotics',
  hideOnScroll: false,
  items: [
    // Left side items
    {
      to: '/',
      label: 'Home',
      position: 'left',
    },
    // Middle/Title area (non-clickable)
    {
      type: 'html',
      value: '<div class="navbar__title">Physical AI & Humanoid Robotics</div>',
      position: 'left',
      className: 'navbar__title-container',
    },
    // Right side items
    {
      type: 'localeDropdown',
      position: 'right',
    },
    {
      type: 'dropdown',
      label: '👤',
      position: 'right',
      items: [
        {
          label: 'Login',
          to: '/login',
        },
        {
          label: 'Signup',
          to: '/signup',
        },
      ],
    },
    {
      type: 'darkModeToggle',
      position: 'right',
    },
  ],
};

// Additional notes for implementation:
/*
1. Make sure to add i18n configuration to docusaurus.config.js:
i18n: {
  defaultLocale: 'en',
  locales: ['en'],
},

2. Create login and signup pages in src/pages/ directory:
- src/pages/login.js
- src/pages/signup.js

3. To ensure the title appears correctly, you might want to hide the logo:
logo: {
  src: '', // empty or a transparent pixel
  alt: 'Physical AI & Humanoid Robotics',
  style: {
    display: 'none' // hide logo to make room for title
  }
},

4. You may need to add some custom CSS to align the title properly:
.navbar__title-container {
  display: flex;
  align-items: center;
  height: 100%;
}

.navbar__title {
  font-weight: bold;
  font-size: 1.2rem;
  margin-left: 1rem;
}
*/
module.exports = navbarConfig;