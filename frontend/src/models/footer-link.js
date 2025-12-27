/**
 * Model representing a footer link
 * @typedef {Object} FooterLink
 * @property {string} id - Unique identifier for the link
 * @property {string} title - Display text for the link
 * @property {string} url - Destination URL
 * @property {'social'|'community'|'legal'} category - Link category
 * @property {string} [icon] - Social media icon identifier
 */

/**
 * Creates a new FooterLink instance
 * @param {Object} params - Parameters for creating a FooterLink
 * @param {string} params.id - Unique identifier for the link
 * @param {string} params.title - Display text for the link
 * @param {string} params.url - Destination URL
 * @param {'social'|'community'|'legal'} params.category - Link category
 * @param {string} [params.icon] - Social media icon identifier
 * @returns {FooterLink} A new FooterLink instance
 */
export const createFooterLink = ({ id, title, url, category, icon = null }) => {
  return {
    id,
    title,
    url,
    category,
    icon
  };
};

/**
 * Validates if an object conforms to the FooterLink structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid FooterLink
 */
export const isValidFooterLink = (obj) => {
  const validCategories = ['social', 'community', 'legal'];
  
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.id === 'string' &&
    typeof obj.title === 'string' &&
    typeof obj.url === 'string' &&
    validCategories.includes(obj.category) &&
    (typeof obj.icon === 'string' || obj.icon === null)
  );
};

export default { createFooterLink, isValidFooterLink };