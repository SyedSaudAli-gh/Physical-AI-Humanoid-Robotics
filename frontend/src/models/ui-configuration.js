/**
 * Model representing UI configuration settings
 * @typedef {Object} UIConfiguration
 * @property {string} theme - Current theme (light/dark/auto)
 * @property {string} fontSize - Current font size setting
 * @property {string} spacing - Current spacing scale
 * @property {boolean} isResponsive - Whether responsive mode is active
 * @property {Object} breakpoints - Device breakpoint settings
 */

/**
 * Creates a new UIConfiguration instance
 * @param {Object} params - Parameters for creating a UIConfiguration
 * @param {string} params.theme - Current theme (light/dark/auto) (defaults to 'light')
 * @param {string} params.fontSize - Current font size setting (defaults to 'medium')
 * @param {string} params.spacing - Current spacing scale (defaults to 'normal')
 * @param {boolean} params.isResponsive - Whether responsive mode is active (defaults to true)
 * @param {Object} params.breakpoints - Device breakpoint settings
 * @returns {UIConfiguration} A new UIConfiguration instance
 */
export const createUIConfiguration = ({ 
  theme = 'light', 
  fontSize = 'medium', 
  spacing = 'normal', 
  isResponsive = true, 
  breakpoints = { xs: 0, sm: 576, md: 768, lg: 992, xl: 1200 } 
}) => {
  return {
    theme,
    fontSize,
    spacing,
    isResponsive,
    breakpoints
  };
};

/**
 * Validates if an object conforms to the UIConfiguration structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid UIConfiguration
 */
export const isValidUIConfiguration = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.theme === 'string' &&
    typeof obj.fontSize === 'string' &&
    typeof obj.spacing === 'string' &&
    typeof obj.isResponsive === 'boolean' &&
    typeof obj.breakpoints === 'object' &&
    obj.breakpoints !== null
  );
};

export default { createUIConfiguration, isValidUIConfiguration };