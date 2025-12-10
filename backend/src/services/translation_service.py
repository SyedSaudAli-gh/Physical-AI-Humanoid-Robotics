from sqlalchemy.orm import Session
from typing import Dict, Any, List, Optional
import json
from ..models.translation_data import TranslationData
from ..models.chapter import Chapter
from ..rag.cohere_embedder import embedding_service
from ...qwen.skills.translation_validator import validate_translation

class TranslationService:
    """
    Service for handling translation using Qwen 1.5 Pro via subagent
    """

    def __init__(self, db: Session):
        self.db = db

    def translate_content(self, content: str, target_language: str = "ur",
                         source_language: str = "en") -> str:
        """
        Translate content to target language using Qwen via subagent
        In a real implementation, this would call the Qwen subagent
        For this implementation, we'll provide placeholder translations for Urdu
        """
        if target_language == "ur":
            # In a real implementation, this would call Qwen via subagent
            # translated_text = self._call_qwen_subagent(content, target_language)
            # For now, we'll return a placeholder
            return self._get_urdu_translation(content)
        else:
            # For other languages, return original content
            return content

    def _get_urdu_translation(self, content: str) -> str:
        """
        Placeholder function to simulate Urdu translation
        In a real implementation, this would call the Qwen subagent
        """
        # This is a simplified placeholder translation
        # In reality, you'd call the Qwen subagent here
        placeholders = {
            "Introduction": "تعارف",
            "ROS 2": "آر او ایس 2",
            "chapter": "چیپٹر",
            "content": "مواد",
            "textbook": " textbook",  # Keep as is since there's no direct translation
            "robotics": "روبوٹکس",
            "module": "ماڈیول",
            "learning": "سیکھنا",
            "outcomes": "نتائج",
            "example": "مثال",
            "code": "کوڈ",
            "simulation": "سمولیشن",
            "practical": "عملی",
            "theoretical": "نظریاتی",
            "foundations": "ادارے",
            "Vision-Language-Action": " وژن-لینگویج-ایکشن",
            "NVIDIA Isaac": "این ویڈیا ایزک",
            "Gazebo": "گزیبو",
            "Unity": "یونٹی",
            "AI": "مصنوعی ذہنیت",
            "artificial intelligence": "مصنوعی ذہنیت",
            "humanoid": "ہیومنوڈ",
            "robot": "روبوٹ"
        }

        translated = content
        for eng, urdu in placeholders.items():
            translated = translated.replace(eng, urdu)

        # Add a note that this is a placeholder translation
        return f"[PLACEHOLDER URDU TRANSLATION] {translated}\n\n[Note: This is a placeholder translation. In a real implementation, this would be translated using Qwen 1.5 Pro via subagent]"

    def store_translation(self, source_content_id: str, target_language: str,
                         translated_content: str, verification_status: str = "pending",
                         verified_by: Optional[str] = None) -> bool:
        """
        Store translated content in the database
        """
        try:
            translation = TranslationData(
                source_content_id=source_content_id,
                target_language=target_language,
                translated_content=translated_content,
                verification_status=verification_status,
                verified_by=verified_by
            )

            self.db.add(translation)
            self.db.commit()
            return True
        except Exception as e:
            print(f"Error storing translation: {str(e)}")
            self.db.rollback()
            return False

    def get_translation(self, source_content_id: str, target_language: str) -> Optional[TranslationData]:
        """
        Get existing translation from database
        """
        return self.db.query(TranslationData).filter(
            TranslationData.source_content_id == source_content_id,
            TranslationData.target_language == target_language
        ).first()

    def translate_and_store(self, source_content_id: str, content: str,
                           target_language: str = "ur") -> Optional[TranslationData]:
        """
        Translate content and store in database
        """
        # Check if translation already exists
        existing = self.get_translation(source_content_id, target_language)
        if existing:
            return existing

        # Translate the content
        translated_content = self.translate_content(content, target_language)

        # Store the translation initially as pending verification
        if self.store_translation(source_content_id, target_language, translated_content, "pending"):
            return self.get_translation(source_content_id, target_language)

        return None

    def get_or_create_translation(self, source_content_id: str, content: str,
                                 target_language: str = "ur") -> str:
        """
        Get existing translation or create a new one
        """
        # First, try to get existing translation
        translation = self.get_translation(source_content_id, target_language)

        if translation:
            return translation.translated_content
        else:
            # Create new translation
            new_translation = self.translate_and_store(source_content_id, content, target_language)
            if new_translation:
                return new_translation.translated_content
            else:
                # If translation fails, return original content
                return content

    def verify_translation(self, translation_id: str, verified_by: Optional[str] = None) -> bool:
        """
        Verify a translation and update its status
        """
        translation = self.db.query(TranslationData).filter(
            TranslationData.id == translation_id
        ).first()

        if not translation:
            return False

        # Get the original content for validation
        # In a real implementation, you'd get this from the source
        # For this example, we'll just validate with the service
        validation_result = validate_translation(
            translation.source_content_id,  # This would be the original text in reality
            translation.translated_content
        )

        # Update the verification status based on validation
        if validation_result.get("is_valid", False):
            translation.verification_status = "verified"
        else:
            translation.verification_status = "needs_revision"

        # Set who verified it
        if verified_by:
            translation.verified_by = verified_by

        try:
            self.db.commit()
            return True
        except Exception as e:
            print(f"Error updating translation verification: {str(e)}")
            self.db.rollback()
            return False

    def get_translations_by_status(self, status: str) -> List[TranslationData]:
        """
        Get all translations with a specific verification status
        """
        return self.db.query(TranslationData).filter(
            TranslationData.verification_status == status
        ).all()

    def update_translation_verification_status(self, translation_id: str, status: str,
                                              verified_by: Optional[str] = None) -> bool:
        """
        Update the verification status of a translation
        """
        translation = self.db.query(TranslationData).filter(
            TranslationData.id == translation_id
        ).first()

        if not translation:
            return False

        translation.verification_status = status
        if verified_by:
            translation.verified_by = verified_by

        try:
            self.db.commit()
            return True
        except Exception as e:
            print(f"Error updating translation status: {str(e)}")
            self.db.rollback()
            return False

# Example usage
if __name__ == "__main__":
    print("Translation service created")