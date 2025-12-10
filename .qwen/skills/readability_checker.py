"""
Qwen CLI Skill for Readability Checking
This module provides functionality to check content readability using Flesch-Kincaid grade level.
Target is grade level ≤ 10 as specified in the requirements.
"""
from typing import Dict, Any
import re

class ReadabilityCheckerSkill:
    """
    Skill for checking readability of content using Flesch-Kincaid grade level
    """
    
    def __init__(self, target_grade_level: float = 10.0):
        self.target_grade_level = target_grade_level
    
    def check_readability(self, text: str) -> Dict[str, Any]:
        """
        Check readability of text using Flesch-Kincaid grade level
        """
        # Calculate Flesch-Kincaid Grade Level
        # Formula: 0.39 * (total words / total sentences) + 11.8 * (total syllables / total words) - 15.59
        sentences = self._count_sentences(text)
        words = self._count_words(text)
        syllables = self._count_syllables(text)
        
        if sentences == 0 or words == 0:
            return {
                "flesch_kincaid_grade": 0,
                "sentence_count": 0,
                "word_count": 0,
                "syllable_count": 0,
                "is_readable": True,
                "readability_status": "Insufficient content to calculate readability",
                "target_grade_level": self.target_grade_level
            }
        
        # Calculate Flesch-Kincaid grade level
        average_words_per_sentence = words / sentences
        average_syllables_per_word = syllables / words
        
        flesch_kincaid_grade = (
            0.39 * average_words_per_sentence + 
            11.8 * average_syllables_per_word - 
            15.59
        )
        
        is_readable = flesch_kincaid_grade <= self.target_grade_level
        
        return {
            "flesch_kincaid_grade": round(flesch_kincaid_grade, 2),
            "sentence_count": sentences,
            "word_count": words,
            "syllable_count": syllables,
            "is_readable": is_readable,
            "readability_status": (
                f"Grade level {flesch_kincaid_grade:.2f} - {'Acceptable' if is_readable else 'Too complex'}" 
                f" (target: ≤{self.target_grade_level})"
            ),
            "target_grade_level": self.target_grade_level
        }
    
    def _count_sentences(self, text: str) -> int:
        """
        Count sentences in text
        """
        # Split by sentence-ending punctuation
        sentences = re.split(r'[.!?]+', text)
        # Filter out empty strings
        sentences = [s.strip() for s in sentences if s.strip()]
        return len(sentences)
    
    def _count_words(self, text: str) -> int:
        """
        Count words in text
        """
        # Remove punctuation and split by whitespace
        words = re.findall(r'\b\w+\b', text.lower())
        return len(words)
    
    def _count_syllables(self, text: str) -> int:
        """
        Count syllables in text (simplified approximation)
        """
        vowels = "aeiouy"
        word_count = 0
        
        words = re.findall(r'\b\w+\b', text.lower())
        for word in words:
            syllable_count = 0
            prev_was_vowel = False
            
            for char in word:
                is_vowel = char in vowels
                if is_vowel and not prev_was_vowel:
                    syllable_count += 1
                prev_was_vowel = is_vowel
            
            # Handle silent 'e' at the end of words
            if word.endswith('e') and syllable_count > 1:
                syllable_count -= 1
            
            # Handle words ending with 'ed' that are not pronounced as extra syllable
            if word.endswith('ed') and len(word) > 2 and word[-3] not in 'td':
                syllable_count -= 1
            
            # Every word has at least one syllable
            if syllable_count == 0:
                syllable_count = 1
                
            word_count += syllable_count
        
        return word_count

def check_readability(text: str) -> Dict[str, Any]:
    """
    Main function to check readability
    This would be called by the Qwen CLI
    """
    checker = ReadabilityCheckerSkill()
    result = checker.check_readability(text)
    return result

if __name__ == "__main__":
    print("Readability Checker Skill initialized")
    
    # Example usage
    sample_text = """
    This is a sample text for readability checking. It contains several sentences 
    and should be evaluated for its Flesch-Kincaid grade level. The target grade 
    level for the Physical AI & Humanoid Robotics textbook is 10 or below.
    """
    
    result = check_readability(sample_text)
    print(f"Readability result: {result}")