import json
import cohere
from typing import Dict, Any
from sqlalchemy.orm import Session
from models.chapter import Chapter
from config import settings


class TranslationService:
    def __init__(self, db: Session):
        self.db = db
        self.cohere_client = cohere.Client(settings.cohere_api_key)

    def translate_chapter_to_urdu(self, chapter_id: int) -> Dict[str, Any]:
        """Translate chapter content to Urdu while preserving technical terms"""

        # Get the chapter content
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")

        # If already translated, return cached version
        if chapter.urdu_translation:
            return {
                "id": chapter.id,
                "title": chapter.title,
                "urdu_content": chapter.urdu_translation
            }

        # Prepare content for translation - preserve technical terms
        content = self._preserve_technical_terms(chapter.content)

        # Use Cohere for translation
        try:
            response = self.cohere_client.translate(
                text=content,
                source_language="en",
                target_language="ur"
            )

            urdu_translation = response.translated_text

            # Restore technical terms in Urdu content
            urdu_translation = self._restore_technical_terms(urdu_translation)

            # Save the translation to the database
            chapter.urdu_translation = urdu_translation
            self.db.commit()

            return {
                "id": chapter.id,
                "title": chapter.title,
                "urdu_content": urdu_translation
            }
        except Exception as e:
            raise Exception(f"Translation failed: {str(e)}")

    def translate_content(self, content: str, target_language: str, source_language: str = "en") -> str:
        """Translate arbitrary content to target language"""
        try:
            response = self.cohere_client.translate(
                text=content,
                source_language=source_language,
                target_language=target_language
            )
            return response.translated_text
        except Exception as e:
            raise Exception(f"Content translation failed: {str(e)}")

    def get_or_create_translation(self, source_content_id: str, content: str, target_language: str) -> str:
        """Get existing translation or create a new one"""
        # For chapter translation, we need to get the chapter by ID
        if source_content_id.startswith('chapter_'):
            chapter_id = int(source_content_id.replace('chapter_', ''))
            chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
            if chapter and chapter.urdu_translation:
                return chapter.urdu_translation

        # If no existing translation, create a new one
        return self.translate_content(content, target_language)

    def store_translation(self, source_content_id: str, target_language: str, translation: str):
        """Store translation in database"""
        # If this is a chapter, store in the chapter model
        if source_content_id.startswith('chapter_'):
            chapter_id = int(source_content_id.replace('chapter_', ''))
            chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
            if chapter:
                if target_language == 'ur':
                    chapter.urdu_translation = translation
                self.db.commit()

    def _preserve_technical_terms(self, content: str) -> str:
        """Identify and preserve technical terms during translation"""
        # In a real implementation, this would identify technical terms and replace them
        # with placeholders to preserve them during translation
        return content

    def _restore_technical_terms(self, translated_content: str) -> str:
        """Restore technical terms after translation"""
        # In a real implementation, this would restore technical terms from placeholders
        return translated_content

    def translate_text_to_urdu(self, text: str) -> str:
        """Translate arbitrary text to Urdu"""
        try:
            response = self.cohere_client.translate(
                text=text,
                source_language="en",
                target_language="ur"
            )
            return response.translated_text
        except Exception as e:
            raise Exception(f"Text translation failed: {str(e)}")