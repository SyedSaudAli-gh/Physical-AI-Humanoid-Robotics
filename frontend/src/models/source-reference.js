/**
 * Model representing a source reference in the chatbot response
 * @typedef {Object} SourceReference
 * @property {string} source_url - URL of the original source
 * @property {string} page_title - Title of the original page
 * @property {string} snippet - Relevant snippet from the source
 * @property {number} relevance_score - Relevance score of this source to the query (0.0 to 1.0)
 */

/**
 * Creates a new SourceReference instance
 * @param {Object} params - Parameters for creating a SourceReference
 * @param {string} params.source_url - URL of the original source
 * @param {string} params.page_title - Title of the original page
 * @param {string} params.snippet - Relevant snippet from the source
 * @param {number} params.relevance_score - Relevance score of this source to the query (0.0 to 1.0)
 * @returns {SourceReference} A new SourceReference instance
 */
export const createSourceReference = ({ source_url, page_title, snippet, relevance_score }) => {
  return {
    source_url,
    page_title,
    snippet,
    relevance_score
  };
};

/**
 * Validates if an object conforms to the SourceReference structure
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid SourceReference
 */
export const isValidSourceReference = (obj) => {
  return (
    typeof obj === 'object' &&
    obj !== null &&
    typeof obj.source_url === 'string' &&
    typeof obj.page_title === 'string' &&
    typeof obj.snippet === 'string' &&
    typeof obj.relevance_score === 'number' &&
    obj.relevance_score >= 0.0 &&
    obj.relevance_score <= 1.0
  );
};

export default { createSourceReference, isValidSourceReference };