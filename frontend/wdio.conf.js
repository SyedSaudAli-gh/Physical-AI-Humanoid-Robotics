// wdio.conf.js - WebdriverIO configuration for cross-browser testing

const browserstack = {
  user: process.env.BROWSERSTACK_USER,
  key: process.env.BROWSERSTACK_ACCESS_KEY,
};

exports.config = {
  user: browserstack.user,
  key: browserstack.key,
  hostname: 'hub.browserstack.com',
  port: 80,
  
  // Test framework
  framework: 'mocha',
  mochaOpts: {
    ui: 'bdd',
    timeout: 60000,
  },
  
  // Browsers to test
  capabilities: [
    {
      browserName: 'chrome',
      browserVersion: 'latest',
      'bstack:options': {
        os: 'Windows',
        osVersion: '11',
        local: 'true',
        seleniumVersion: '4.0.0',
      },
    },
    {
      browserName: 'firefox',
      browserVersion: 'latest',
      'bstack:options': {
        os: 'Windows',
        osVersion: '11',
        local: 'true',
        seleniumVersion: '4.0.0',
      },
    },
    {
      browserName: 'safari',
      browserVersion: 'latest',
      'bstack:options': {
        os: 'OS X',
        osVersion: 'Monterey',
        local: 'true',
        seleniumVersion: '4.0.0',
      },
    },
    {
      browserName: 'edge',
      browserVersion: 'latest',
      'bstack:options': {
        os: 'Windows',
        osVersion: '11',
        local: 'true',
        seleniumVersion: '4.0.0',
      },
    },
  ],
  
  // Test files
  specs: [
    './tests/e2e/**/*.spec.js'
  ],
  
  // Hooks
  onPrepare: function (config, capabilities) {
    console.log('Connecting to BrowserStack...');
  },
  
  onComplete: function (exitCode, config, capabilities, results) {
    console.log('Tests completed!');
  }
};