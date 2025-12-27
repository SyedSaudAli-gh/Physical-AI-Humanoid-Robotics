/**
 * Model representing the current page context
 * @typedef {Object} PageContext
 * @property {string} moduleId - Identifier for the current module
 * @property {string} chapterId - Identifier for the current chapter
 * @property {string} pageTitle - Current page title
 * @property {string} pageUrl - Current page URL
 * @property {Array<Object>} breadcrumbs - Navigation breadcrumb trail
 */

/**
 * Creates a new PageContext instance
 * @param {Object} params - Parameters for creating a PageContext
 * @param {string} params.moduleId - Identifier for the current module
 * @param {string} params.chapterId - Identifier for the current chapter
 * @param {string} params.pageTitle - Current page title
 * @param {string} params.pageUrl - Current page URL
 * @param {Array<Object>} params.breadcrumbs - Navigation breadcrumb trail
 * @returns {PageContext} A new PageContext instance
 */
export const createPageContext = ({ moduleId, chapterId, pageTitle, pageUrl, breadcrumbs = [] }) => {
  return {
    moduleId,
    chapterId,
    pageTitle,
    pageUrl,
    breadcrumbs
  };
};

/**
 * Validates if an object conforms to the PageContext structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid PageContext
 */
export const isValidPageContext = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.moduleId === 'string' &&
    typeof obj.chapterId === 'string' &&
    typeof obj.pageTitle === 'string' &&
    typeof obj.pageUrl === 'string' &&
    Array.isArray(obj.breadcrumbs)
  );
};

export default { createPageContext, isValidPageContext };