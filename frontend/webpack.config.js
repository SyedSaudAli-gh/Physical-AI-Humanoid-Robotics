module.exports = function (config, options, webpack) {
  // Modify the devServer configuration to add proxy
  if (options.serve) {
    config.devServer = config.devServer || {};
    config.devServer.proxy = {
      '/api': {
        target: 'http://localhost:8000',
        changeOrigin: true,
        secure: false,
      },
    };
  }

  return config;
};