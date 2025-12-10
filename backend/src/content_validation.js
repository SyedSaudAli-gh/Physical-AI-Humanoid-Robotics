// Content validation utility to ensure chapters meet requirements
// This would be used both in the backend validation and potentially as a CLI tool

class ContentValidator {
  /**
   * Count words in a text content
   * @param {string} text - The text to count words in
   * @return {number} The number of words
   */
  static countWords(text) {
    if (!text || typeof text !== 'string') {
      return 0;
    }
    
    // Remove HTML tags and extra whitespace
    const cleanText = text.replace(/<[^>]*>/g, ' ').replace(/\s+/g, ' ').trim();
    
    // Split by spaces and count non-empty words
    return cleanText.split(/\s+/).filter(word => word.length > 0).length;
  }

  /**
   * Validate that content meets word count requirements
   * @param {string} content - The content to validate
   * @param {number} minWords - Minimum number of words (default: 2000)
   * @param {number} maxWords - Maximum number of words (default: 4000)
   * @return {object} Validation result with isValid and message
   */
  static validateWordCount(content, minWords = 2000, maxWords = 4000) {
    const wordCount = this.countWords(content);
    
    if (wordCount < minWords) {
      return {
        isValid: false,
        wordCount: wordCount,
        message: `Content is too short: ${wordCount} words. Minimum required: ${minWords} words.`
      };
    }
    
    if (wordCount > maxWords) {
      return {
        isValid: false,
        wordCount: wordCount,
        message: `Content is too long: ${wordCount} words. Maximum allowed: ${maxWords} words.`
      };
    }
    
    return {
      isValid: true,
      wordCount: wordCount,
      message: `Content meets requirements: ${wordCount} words.`
    };
  }

  /**
   * Validate chapter content structure
   * @param {object} chapter - The chapter object to validate
   * @return {object} Validation result
   */
  static validateChapter(chapter) {
    const results = {
      isValid: true,
      issues: []
    };

    // Check if chapter has required fields
    if (!chapter.title || !chapter.content) {
      results.isValid = false;
      results.issues.push("Chapter is missing required fields (title or content)");
      return results;
    }

    // Validate word count
    const wordValidation = this.validateWordCount(chapter.content);
    if (!wordValidation.isValid) {
      results.isValid = false;
      results.issues.push(wordValidation.message);
    }

    // Check for learning outcomes
    if (!chapter.learning_outcomes || chapter.learning_outcomes.length === 0) {
      results.issues.push("Chapter is missing learning outcomes");
    }

    // Check for theoretical foundations section
    if (!chapter.content.includes("Theoretical Foundations")) {
      results.issues.push("Chapter is missing Theoretical Foundations section");
    }

    // Check for practical simulations section
    if (!chapter.content.includes("Practical Simulations")) {
      results.issues.push("Chapter is missing Practical Simulations section");
    }

    // Check for code examples section
    if (!chapter.content.toLowerCase().includes("code example")) {
      results.issues.push("Chapter is missing Code Examples section");
    }

    return results;
  }
}

// Example usage in a CLI tool context
if (require.main === module) {
  // Example of how to validate a chapter
  const sampleChapter = {
    title: "Sample Chapter",
    content: `
# Sample Chapter
      
## Theoretical Foundations
This section covers the theoretical foundations...

## Practical Simulations
This section covers practical simulations...

## Code Examples
Here are some code examples...
    `.repeat(200), // Repeat to ensure it's above 2000 words
    learning_outcomes: ["Understand basic concepts", "Apply theoretical knowledge"]
  };

  const validation = ContentValidator.validateChapter(sampleChapter);
  console.log("Validation result:", validation);
}

module.exports = ContentValidator;