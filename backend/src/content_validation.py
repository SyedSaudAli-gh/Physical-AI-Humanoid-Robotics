# Content validation utility to ensure chapters meet requirements
import re
from typing import Dict, List, Tuple

class ContentValidator:
    """
    Utility class to validate content meets specified requirements
    """
    
    @staticmethod
    def count_words(text: str) -> int:
        """
        Count words in text content
        """
        if not text or not isinstance(text, str):
            return 0
        
        # Remove markdown formatting and extra whitespace
        clean_text = re.sub(r'[*_`#\-]|[!\[]', ' ', text)
        clean_text = re.sub(r'\s+', ' ', clean_text).strip()
        
        # Split by spaces and count non-empty words
        return len([word for word in clean_text.split() if word.strip()])
    
    @staticmethod
    def validate_word_count(content: str, min_words: int = 2000, max_words: int = 4000) -> Dict:
        """
        Validate that content meets word count requirements
        """
        word_count = ContentValidator.count_words(content)
        
        if word_count < min_words:
            return {
                'is_valid': False,
                'word_count': word_count,
                'message': f'Content is too short: {word_count} words. Minimum required: {min_words} words.'
            }
        
        if word_count > max_words:
            return {
                'is_valid': False,
                'word_count': word_count,
                'message': f'Content is too long: {word_count} words. Maximum allowed: {max_words} words.'
            }
        
        return {
            'is_valid': True,
            'word_count': word_count,
            'message': f'Content meets requirements: {word_count} words.'
        }
    
    @staticmethod
    def validate_chapter(chapter: Dict) -> Dict:
        """
        Validate chapter content structure and requirements
        """
        results = {
            'is_valid': True,
            'issues': []
        }
        
        # Check if chapter has required fields
        if not chapter.get('title') or not chapter.get('content'):
            results['is_valid'] = False
            results['issues'].append("Chapter is missing required fields (title or content)")
            return results
        
        # Validate word count
        word_validation = ContentValidator.validate_word_count(chapter['content'])
        if not word_validation['is_valid']:
            results['is_valid'] = False
            results['issues'].append(word_validation['message'])
        
        # Check for learning outcomes
        if not chapter.get('learning_outcomes') or len(chapter.get('learning_outcomes', [])) == 0:
            results['issues'].append("Chapter is missing learning outcomes")
        
        # Check for theoretical foundations section
        if 'theoretical foundations' not in chapter['content'].lower():
            results['issues'].append("Chapter is missing Theoretical Foundations section")
        
        # Check for practical simulations section
        if 'practical simulations' not in chapter['content'].lower():
            results['issues'].append("Chapter is missing Practical Simulations section")
        
        # Check for code examples section
        if 'code example' not in chapter['content'].lower():
            results['issues'].append("Chapter is missing Code Examples section")
        
        return results

# Example usage
if __name__ == "__main__":
    # Example of how to validate a chapter
    sample_chapter = {
        'title': 'Sample Chapter',
        'content': '''
# Sample Chapter

## Theoretical Foundations
This section covers the theoretical foundations of the topic.
It explains the core concepts and principles that underlie the subject matter.
There are many important aspects to consider when understanding this theory.

## Practical Simulations
This section covers practical simulations that demonstrate the concepts.
We'll go through hands-on examples that illustrate how the theory applies in practice.
These simulations help connect abstract concepts to concrete implementations.

## Code Examples
Here are some code examples that demonstrate the practical applications:
```python
print("Hello, World!")
```

The code examples provide practical illustrations of the concepts discussed.
        ''' * 50,  # Repeat content to ensure it's above 2000 words
        'learning_outcomes': ['Understand basic concepts', 'Apply theoretical knowledge']
    }
    
    validation = ContentValidator.validate_chapter(sample_chapter)
    print("Validation result:", validation)