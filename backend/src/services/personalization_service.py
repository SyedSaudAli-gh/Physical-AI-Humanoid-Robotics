import json
from typing import Dict, Any, List
from sqlalchemy.orm import Session
from models.chapter import Chapter
from models.user_profile import UserProfile


class PersonalizationService:
    def __init__(self, db: Session):
        self.db = db

    def get_personalized_chapter_content(self, chapter_id: int, user_id: str) -> Dict[str, Any]:
        """Get personalized chapter content based on user profile"""
        
        # Get the chapter content
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")
            
        # Get user profile
        user_profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
        if not user_profile:
            raise ValueError(f"User profile with ID {user_id} not found")
            
        # Determine content based on user's experience level
        content = chapter.content
        
        # If chapter has content variants, adapt based on user's level
        if chapter.content_variants:
            try:
                variants = json.loads(chapter.content_variants)
                
                # Determine user's experience level (default to intermediate if not set)
                user_level = user_profile.experience_level or "intermediate"
                
                # Adjust content based on user level
                if user_level == "beginner":
                    content = variants.get("beginner", content)
                elif user_level == "advanced":
                    content = variants.get("advanced", content)
                else:
                    content = variants.get("intermediate", content)
            except json.JSONDecodeError:
                # If JSON parsing fails, use original content
                pass
                
        # Apply personalization based on user's technical skills
        personalization_applied = self._apply_technical_background_personalization(
            content, user_profile.technical_skills or []
        )
        
        return {
            "id": chapter.id,
            "title": chapter.title,
            "content": personalization_applied,
            "original_content": chapter.content,
            "user_experience_level": user_profile.experience_level,
            "adapted_for_skills": user_profile.technical_skills
        }

    def _apply_technical_background_personalization(self, content: str, user_skills: List[str]) -> str:
        """Adjust content based on user's technical background knowledge"""
        
        # If user has advanced skills, provide more concise explanations
        if "advanced" in str(user_skills).lower() or len([skill for skill in user_skills if skill.lower() in 
            ["ros", "nvidia-isaac", "gazebo", "unity", "python", "cpp", "machine-learning", "computer-vision"]]) >= 3:
            
            # Add more advanced content or skip basic explanations if they exist in the content
            content += "\n\n> Pro Tip: Since you have advanced experience, consider exploring advanced configuration files and optimizations."
        
        # If user is a beginner, add more explanations
        elif "beginner" in str(user_skills).lower() or len(user_skills) == 0:
            content += "\n\n> Beginner Tip: For better understanding, try running the code examples in a simulation environment first before implementing on physical robots."
            
        return content