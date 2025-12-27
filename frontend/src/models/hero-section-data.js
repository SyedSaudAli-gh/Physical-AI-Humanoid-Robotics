/**
 * Model representing hero section data
 * @typedef {Object} HeroSectionData
 * @property {string} title - Main heading for the hero section
 * @property {string} subtitle - Supporting text for the hero section
 * @property {string} imageUrl - Path to the AI-generated hero image
 * @property {string} ctaText - Call-to-action button text
 * @property {string} ctaUrl - Call-to-action destination
 * @property {Array<ModuleCard>} moduleCards - Navigation cards for modules
 */

/**
 * Creates a new HeroSectionData instance
 * @param {Object} params - Parameters for creating a HeroSectionData
 * @param {string} params.title - Main heading for the hero section
 * @param {string} params.subtitle - Supporting text for the hero section
 * @param {string} params.imageUrl - Path to the AI-generated hero image
 * @param {string} params.ctaText - Call-to-action button text
 * @param {string} params.ctaUrl - Call-to-action destination
 * @param {Array<ModuleCard>} params.moduleCards - Navigation cards for modules
 * @returns {HeroSectionData} A new HeroSectionData instance
 */
export const createHeroSectionData = ({ title, subtitle, imageUrl, ctaText, ctaUrl, moduleCards = [] }) => {
  return {
    title,
    subtitle,
    imageUrl,
    ctaText,
    ctaUrl,
    moduleCards
  };
};

/**
 * Validates if an object conforms to the HeroSectionData structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid HeroSectionData
 */
export const isValidHeroSectionData = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.title === 'string' &&
    typeof obj.subtitle === 'string' &&
    typeof obj.imageUrl === 'string' &&
    typeof obj.ctaText === 'string' &&
    typeof obj.ctaUrl === 'string' &&
    Array.isArray(obj.moduleCards)
  );
};

export default { createHeroSectionData, isValidHeroSectionData };