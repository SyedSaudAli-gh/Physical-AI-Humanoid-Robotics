from sqlalchemy.orm import Session
from typing import Dict, Any, Optional
import json
from ..models.user_profile import User
from ..models.chapter import Chapter

class PersonalizationService:
    """
    Service for handling content personalization based on user skills and preferences
    """
    
    def __init__(self, db: Session):
        self.db = db
    
    def get_personalized_content(self, chapter_id: str, user_id: str) -> Dict[str, Any]:
        """
        Get chapter content personalized for the user based on their skills and preferences
        """
        # Get the chapter
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with id {chapter_id} not found")
        
        # Get user profile
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            # If no user profile, return default content
            return {
                'id': chapter.id,
                'title': chapter.title,
                'content': chapter.content,
                'module_id': chapter.module_id,
                'personalization_applied': False,
                'message': 'No user profile found, showing default content'
            }
        
        # Parse content variants if they exist
        content_variants = {}
        if chapter.content_variants:
            try:
                content_variants = json.loads(chapter.content_variants)
            except json.JSONDecodeError:
                # If JSON parsing fails, continue with empty variants
                content_variants = {}
        
        # Determine which content to return based on user preferences
        preferred_content = chapter.content  # Default to base content
        
        # Check for difficulty override in user preferences
        difficulty_override = None
        if user.preferences and isinstance(user.preferences, dict):
            difficulty_override = user.preferences.get('chapter_difficulty_override')
        
        # If there's an override, try to use that content variant
        if difficulty_override and content_variants and content_variants.get(difficulty_override):
            preferred_content = content_variants[difficulty_override]
        # Otherwise, try to match content to user's experience level
        elif user.experience_level and content_variants and content_variants.get(user.experience_level):
            preferred_content = content_variants[user.experience_level]
        
        # Language preference
        lang_content = preferred_content
        if user.preferences and user.preferences.get('preferred_language') == 'ur':
            if content_variants and content_variants.get('urdu'):
                lang_content = content_variants['urdu']
        
        return {
            'id': chapter.id,
            'title': chapter.title,
            'content': lang_content,
            'module_id': chapter.module_id,
            'personalization_applied': True,
            'user_profile_used': {
                'experience_level': user.experience_level,
                'technical_skills': user.technical_skills,
                'preferences': user.preferences
            }
        }
    
    def generate_content_variants(self, chapter_id: str, base_content: str) -> bool:
        """
        Generate different difficulty levels of content based on base content
        In a real implementation, this would use AI to generate variants
        """
        # In a real implementation, this would call an AI service to generate
        # different versions of the content based on difficulty level
        # For now, we'll just store the base content as all variants
        
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            return False
        
        # This is a simplified implementation - in reality, AI would generate
        # different versions of the content for each difficulty level
        content_variants = {
            'beginner': self._simplify_content(base_content),
            'intermediate': base_content,  # Default content
            'advanced': self._enrich_content(base_content),
            'urdu': 'یہ چیپٹر کا اردو ورژن ہے'  # Placeholder Urdu translation
        }
        
        # Store the variants in the chapter
        chapter.content_variants = json.dumps(content_variants)
        
        try:
            self.db.commit()
            return True
        except Exception:
            self.db.rollback()
            return False
    
    def _simplify_content(self, content: str) -> str:
        """
        Simplify content for beginner level (in a real implementation)
        """
        # This would use NLP techniques to simplify content
        # For now, return the original content with a note
        return f"[BEGINNER LEVEL] {content}"
    
    def _enrich_content(self, content: str) -> str:
        """
        Enrich content for advanced level (in a real implementation)
        """
        # This would add more complex examples, deeper explanations, etc.
        # For now, return the original content with a note
        return f"[ADVANCED LEVEL] {content} [Additional advanced material would go here]"

# Example usage
if __name__ == "__main__":
    print("Personalization service created")