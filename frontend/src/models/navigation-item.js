/**
 * Model representing a navigation item in the UI
 * @typedef {Object} NavigationItem
 * @property {string} id - Unique identifier for the navigation item
 * @property {string} title - Display text for the navigation item
 * @property {string} url - Destination URL for the navigation item
 * @property {string} [icon] - Icon identifier for the navigation item
 * @property {boolean} isExternal - Whether the link opens in a new tab
 */

/**
 * Creates a new NavigationItem instance
 * @param {Object} params - Parameters for creating a NavigationItem
 * @param {string} params.id - Unique identifier for the navigation item
 * @param {string} params.title - Display text for the navigation item
 * @param {string} params.url - Destination URL for the navigation item
 * @param {string} [params.icon] - Icon identifier for the navigation item
 * @param {boolean} params.isExternal - Whether the link opens in a new tab
 * @returns {NavigationItem} A new NavigationItem instance
 */
export const createNavigationItem = ({ id, title, url, icon = null, isExternal = false }) => {
  return {
    id,
    title,
    url,
    icon,
    isExternal
  };
};

/**
 * Validates if an object conforms to the NavigationItem structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid NavigationItem
 */
export const isValidNavigationItem = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.id === 'string' &&
    typeof obj.title === 'string' &&
    typeof obj.url === 'string' &&
    (typeof obj.icon === 'string' || obj.icon === null) &&
    typeof obj.isExternal === 'boolean'
  );
};

export default { createNavigationItem, isValidNavigationItem };