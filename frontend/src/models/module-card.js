/**
 * Model representing a module card
 * @typedef {Object} ModuleCard
 * @property {string} id - Unique identifier for the module card
 * @property {string} title - Module title
 * @property {string} description - Brief description of the module
 * @property {string} [icon] - Icon identifier for the module
 * @property {string} url - URL to the module's main page
 * @property {number} chapterCount - Number of chapters in the module
 */

/**
 * Creates a new ModuleCard instance
 * @param {Object} params - Parameters for creating a ModuleCard
 * @param {string} params.id - Unique identifier for the module card
 * @param {string} params.title - Module title
 * @param {string} params.description - Brief description of the module
 * @param {string} [params.icon] - Icon identifier for the module
 * @param {string} params.url - URL to the module's main page
 * @param {number} params.chapterCount - Number of chapters in the module
 * @returns {ModuleCard} A new ModuleCard instance
 */
export const createModuleCard = ({ id, title, description, icon = null, url, chapterCount }) => {
  return {
    id,
    title,
    description,
    icon,
    url,
    chapterCount
  };
};

/**
 * Validates if an object conforms to the ModuleCard structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid ModuleCard
 */
export const isValidModuleCard = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.id === 'string' &&
    typeof obj.title === 'string' &&
    typeof obj.description === 'string' &&
    (typeof obj.icon === 'string' || obj.icon === null) &&
    typeof obj.url === 'string' &&
    typeof obj.chapterCount === 'number'
  );
};

export default { createModuleCard, isValidModuleCard };