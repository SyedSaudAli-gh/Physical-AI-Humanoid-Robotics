"""
Qwen CLI Skill for Translation Validation
This module provides validation for translated content
"""
from typing import Dict, Any, List
import re

class TranslationValidatorSkill:
    """
    Skill for validating the quality and accuracy of translations
    """
    
    def __init__(self):
        # Define validation rules
        self.validation_rules = {
            "urdu_pattern": r"[\u0600-\u06FF\s\d.,;:!?()\"'\\-]+",  # Arabic/Persian script used in Urdu
            "minimum_translation_ratio": 0.7,  # At least 70% of content should be in target script
            "forbidden_patterns": [  # Patterns that shouldn't appear in good translations
                r"\[PLACEHOLDER.*?\]", 
                r"\[TRANSLATION.*?\]",
                r"ORIGINAL_CONTENT_HERE"
            ]
        }
    
    def validate_urdu_translation(self, original_text: str, translated_text: str) -> Dict[str, Any]:
        """
        Validate the quality of a Urdu translation
        """
        validation_results = {
            "is_valid": True,
            "issues": [],
            "quality_score": 0.0,
            "details": {}
        }
        
        # Check if translated content contains Urdu script
        urdu_chars = re.findall(self.validation_rules["urdu_pattern"], translated_text)
        if urdu_chars:
            urdu_content_ratio = len("".join(urdu_chars)) / len(translated_text) if translated_text else 0
            validation_results["details"]["urdu_content_ratio"] = urdu_content_ratio
            
            if urdu_content_ratio < self.validation_rules["minimum_translation_ratio"]:
                validation_results["is_valid"] = False
                validation_results["issues"].append(
                    f"Urdu content ratio too low: {urdu_content_ratio:.2%}, minimum required: {self.validation_rules['minimum_translation_ratio']:.0%}"
                )
        else:
            validation_results["is_valid"] = False
            validation_results["issues"].append("No Urdu/Persian script characters found in translation")
        
        # Check for forbidden patterns
        for pattern in self.validation_rules["forbidden_patterns"]:
            if re.search(pattern, translated_text, re.IGNORECASE):
                validation_results["is_valid"] = False
                validation_results["issues"].append(f"Translation contains forbidden pattern: {pattern}")
        
        # Calculate quality score based on various factors
        score = self._calculate_quality_score(original_text, translated_text, validation_results)
        validation_results["quality_score"] = score
        
        # Update validity based on quality score
        if score < 0.5:
            validation_results["is_valid"] = False
            validation_results["issues"].append(f"Quality score too low: {score:.2%}")
        
        return validation_results
    
    def _calculate_quality_score(self, original: str, translated: str, validation_results: Dict[str, Any]) -> float:
        """
        Calculate a quality score based on various metrics
        """
        # Base score calculation
        # Consider length similarity (translations are often similar length to original)
        if len(original) > 0 and len(translated) > 0:
            length_similarity = min(len(translated) / len(original), len(original) / len(translated))
        else:
            length_similarity = 0
        
        # Consider content uniqueness (avoid direct copying)
        content_overlap = self._calculate_content_overlap(original, translated)
        
        # Consider language-specific rules
        # For Urdu, ensure proper script usage
        urdu_ratio = validation_results["details"].get("urdu_content_ratio", 0)
        
        # Weighted score calculation
        score = (
            0.4 * length_similarity +
            0.3 * urdu_ratio +
            0.3 * (1 - content_overlap)  # Higher score if less overlap (meaningful translation)
        )
        
        return score
    
    def _calculate_content_overlap(self, original: str, translated: str) -> float:
        """
        Calculate how much of the translated content overlaps with the original
        """
        orig_words = set(original.lower().split())
        trans_words = set(translated.lower().split())
        
        if not orig_words:
            return 0
        
        common_words = orig_words.intersection(trans_words)
        return len(common_words) / len(orig_words)

# Example usage function
def validate_translation(original: str, translated: str) -> Dict[str, Any]:
    """
    Main function to validate translation quality
    This would be called by the Qwen CLI
    """
    validator = TranslationValidatorSkill()
    result = validator.validate_urdu_translation(original, translated)
    return result

if __name__ == "__main__":
    print("Translation Validator Skill initialized")
    
    # Example usage
    original = "This is a test for translation validation."
    translated = "یہ ترجمہ کی توثیق کے لئے ایک ٹیسٹ ہے۔"
    
    result = validate_translation(original, translated)
    print(f"Original: {original}")
    print(f"Translated: {translated}")
    print(f"Validation Result: {result}")