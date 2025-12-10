# Zero-plagiarism detection and content validation infrastructure
import hashlib
import re
from typing import List, Dict, Tuple
from difflib import SequenceMatcher

class PlagiarismDetector:
    """
    Plagiarism detection system to ensure all content is original
    """
    
    def __init__(self):
        self.content_database = {}  # In a real implementation, this would be a database
        self.min_similarity_threshold = 0.8  # 80% similarity considered plagiarism
    
    def add_source_content(self, source_id: str, content: str):
        """
        Add content from an authoritative source to the database
        """
        # Normalize the content for comparison
        normalized = self._normalize_content(content)
        self.content_database[source_id] = normalized
    
    def check_plagiarism(self, new_content: str) -> Tuple[bool, List[Dict[str, any]]]:
        """
        Check if the new content contains plagiarized parts
        Returns: (is_original, list_of_matches)
        """
        new_normalized = self._normalize_content(new_content)
        matches = []
        
        for source_id, source_content in self.content_database.items():
            # Find similar segments
            similarity = self._calculate_similarity(new_normalized, source_content)
            
            if similarity > self.min_similarity_threshold:
                matches.append({
                    'source_id': source_id,
                    'similarity': similarity,
                    'message': f'High similarity ({similarity:.2%}) detected with source {source_id}'
                })
        
        return len(matches) == 0, matches
    
    def _normalize_content(self, content: str) -> str:
        """
        Normalize content by removing extra whitespace, converting to lowercase, etc.
        """
        # Remove extra whitespace and normalize
        content = re.sub(r'\s+', ' ', content.strip().lower())
        # Remove common articles and stop words for more accurate comparison
        content = re.sub(r'\b(a|an|the|and|or|but|in|on|at|to|for|of|with|by|is|are|was|were|be|been|being|have|has|had|do|does|did|will|would|could|should)\b', ' ', content)
        # Remove special characters but keep alphanumeric and spaces
        content = re.sub(r'[^a-z0-9\s]', ' ', content)
        # Remove extra spaces again after processing
        content = re.sub(r'\s+', ' ', content.strip())
        
        return content
    
    def _calculate_similarity(self, content1: str, content2: str) -> float:
        """
        Calculate similarity between two content strings
        """
        return SequenceMatcher(None, content1, content2).ratio()

class ContentValidator:
    """
    Content validation infrastructure to ensure quality and originality
    """
    
    def __init__(self):
        self.plagiarism_detector = PlagiarismDetector()
        self.readability_threshold = 10.0  # Flesch-Kincaid grade level
    
    def validate_content(self, content: str, content_type: str = "textbook") -> Dict[str, any]:
        """
        Validate content for originality, readability, and quality
        """
        results = {
            'is_valid': True,
            'issues': [],
            'warnings': [],
            'content_type': content_type
        }
        
        # Check for plagiarism
        is_original, plagiarism_matches = self.plagiarism_detector.check_plagiarism(content)
        if not is_original:
            results['is_valid'] = False
            results['issues'].extend(plagiarism_matches)
        
        # Check readability (simplified - a real implementation would use a proper readability library)
        readability_score = self._calculate_readability(content)
        if readability_score > self.readability_threshold:
            results['warnings'].append({
                'type': 'readability',
                'message': f'Readability score ({readability_score:.1f}) exceeds target ({self.readability_threshold}). Consider simplifying the language.'
            })
        
        # Check content length requirements (for textbook chapters)
        if content_type == "textbook" or content_type == "chapter":
            word_count = len(content.split())
            if word_count < 2000:
                results['warnings'].append({
                    'type': 'length',
                    'message': f'Content is shorter than minimum required length (currently {word_count} words, minimum 2000)'
                })
            elif word_count > 4000:
                results['warnings'].append({
                    'type': 'length',
                    'message': f'Content exceeds maximum recommended length (currently {word_count} words, maximum 4000)'
                })
        
        return results
    
    def _calculate_readability(self, content: str) -> float:
        """
        Simplified readability calculation (in reality you'd use a proper library)
        This is a placeholder that returns a fixed score for demonstration
        """
        # A real implementation would calculate Flesch-Kincaid grade level
        # For now, return a random score within an appropriate range
        import random
        return random.uniform(6.0, 12.0)  # Return a random score between 6 and 12

# Example usage
if __name__ == "__main__":
    validator = ContentValidator()
    
    # Example content to validate
    sample_content = """
    This is sample content for the Physical AI & Humanoid Robotics textbook.
    It discusses various concepts related to robotic systems, including ROS 2,
    which serves as the robotic nervous system for humanoid robots.
    """
    
    result = validator.validate_content(sample_content, "chapter")
    print(f"Content validation result: {result}")