from sqlalchemy.orm import Session
from typing import Dict, Any, Optional
from ..models.chapter import Chapter
from ..models.module import Module
from ...qwen.subagents.textbook_generator import run_content_generation
from ..services.translation_service import TranslationService

class ContentGenerationService:
    """
    Service for generating textbook content using Qwen CLI subagents
    """
    
    def __init__(self, db: Session):
        self.db = db
        self.translation_service = TranslationService(db)
    
    def generate_chapter_content(self, 
                                module_id: str, 
                                title: str, 
                                topic: str,
                                target_audience: str = "undergraduate_students",
                                word_count: int = 3000) -> Optional[Chapter]:
        """
        Generate chapter content using Qwen subagent and create a new Chapter
        """
        try:
            # Generate content using the Qwen subagent
            content_data = run_content_generation(
                module_id=module_id,
                title=title,
                topic=topic,
                target_audience=target_audience,
                word_count=word_count
            )
            
            # Get the module to associate with the chapter
            module = self.db.query(Module).filter(Module.id == module_id).first()
            if not module:
                raise ValueError(f"Module with ID {module_id} not found")
            
            # Create a new chapter
            chapter = Chapter(
                module_id=module_id,
                title=title,
                content=content_data['content'],
                content_variants=json.dumps(content_data['content_variants']),
                urdu_translation=content_data.get('urdu_translation', '')
            )
            self.db.add(chapter)
            self.db.commit()
            self.db.refresh(chapter)
            
            return chapter
        except Exception as e:
            raise Exception(f"Chapter content generation failed: {str(e)}")
    
    def generate_content_variants(self, chapter_id: str, base_content: str):
        """
        Generate content variants for different difficulty levels
        """
        content_variants = {
            'beginner': self._simplify_content(base_content),
            'intermediate': base_content,  # Default content
            'advanced': self._enrich_content(base_content)
        }
        
        # Update the chapter's content variants
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if chapter:
            import json
            chapter.content_variants = json.dumps(content_variants)
            self.db.commit()
    
    def _generate_urdu_translation(self, chapter_id: str, content: str):
        """
        Generate and store Urdu translation of the content
        """
        # Use the translation service to create Urdu translation
        self.translation_service.translate_and_store(
            source_content_id=chapter_id,
            content=content,
            target_language="ur"
        )
    
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
    print("Content Generation Service created")