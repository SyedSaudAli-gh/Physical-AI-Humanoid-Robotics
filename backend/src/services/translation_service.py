import json
from typing import Dict, Any
from sqlalchemy.orm import Session
from ..models.chapter import Chapter
from ..models.translation_data import TranslationData
from ..config import settings
import re

class TranslationService:
    def __init__(self, db: Session):
        self.db = db

    def translate_chapter_to_urdu(self, chapter_id: int) -> Dict[str, Any]:
        """Translate chapter content to Urdu while preserving technical terms"""

        # Get the chapter content
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")

        # Check for existing translation in the translation_data table
        existing_translation = self.db.query(TranslationData).filter(
            TranslationData.source_content_id == f"chapter_{chapter_id}",
            TranslationData.target_language == "ur"
        ).first()

        if existing_translation:
            return {
                "id": chapter.id,
                "title": chapter.title,
                "urdu_content": existing_translation.translated_content
            }

        # Prepare content for translation - preserve technical terms
        content = self._preserve_technical_terms(chapter.content)

        # Use Qwen subagent for translation
        try:
            urdu_translation = translate_text(content, target_language="ur")  # Qwen subagent call
            urdu_translation = self._restore_technical_terms(urdu_translation)

            # Save the translation in the translation_data table
            translation_data = TranslationData(
                source_content_id=f"chapter_{chapter_id}",
                target_language="ur",
                translated_content=urdu_translation
            )
            self.db.add(translation_data)
            self.db.commit()

            return {
                "id": chapter.id,
                "title": chapter.title,
                "urdu_content": urdu_translation
            }
        except Exception as e:
            raise Exception(f"Chapter translation failed: {str(e)}")

    def _preserve_technical_terms(self, content: str) -> str:
        """Preserve technical terms during translation"""
        technical_terms = [
            "ROS 2", "Gazebo", "Unity", "NVIDIA Isaac", "VLA",
            "Vision-Language-Action", "Embodied Intelligence", "Humanoid Robotics",
            "Physical AI", "Simulation", "Digital Twin", "Perception", "Navigation",
            "Cognitive Planning", "Voice-to-Action", "Reinforcement Learning",
            "Inverse Kinematics", "Forward Kinematics", "Quaternion", "Euler Angles",
            "URDF", "SDF", "Robotics", "Humanoid", "Physical AI", "Embodied AI",
            "Computer Vision", "Motion Planning", "Path Planning", "Trajectory",
            "Control Theory", "Actuator", "Sensor Fusion", "LIDAR", "Computer Vision",
            "Reinforcement Learning"
        ]

        # Replace technical terms with placeholders to preserve them during translation
        preserved_content = content
        self.placeholder_map = {}

        for i, term in enumerate(technical_terms):
            placeholder = f"__TECH_TERM_{i}__"
            self.placeholder_map[placeholder] = term
            preserved_content = re.sub(rf'\b{re.escape(term)}\b', placeholder, preserved_content, flags=re.IGNORECASE)

        return preserved_content

    def _restore_technical_terms(self, translated_content: str) -> str:
        """Restore technical terms after translation"""
        # Restore technical terms from placeholders
        restored_content = translated_content
        for placeholder, term in self.placeholder_map.items():
            restored_content = restored_content.replace(placeholder, term)

        return restored_content

    def translate_text_to_urdu(self, text: str) -> str:
        """Translate arbitrary text to Urdu"""
        try:
            return translate_text(text, target_language="ur")  # Qwen subagent call
        except Exception as e:
            raise Exception(f"Text translation failed: {str(e)}")