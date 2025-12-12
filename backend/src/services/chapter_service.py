from sqlalchemy.orm import Session
from typing import List, Optional
from ..models.chapter import Chapter
from ..models.content import BookContent
from pydantic import BaseModel
from datetime import datetime

class ChapterCreate(BaseModel):
    module_id: str
    title: str
    content: str
    order_index: int
    learning_outcomes: List[str]
    content_variants: Optional[dict] = {}

class ChapterResponse(BaseModel):
    id: str
    module_id: str
    title: str
    content_preview: str  # Only preview for the list view
    order_index: int
    learning_outcomes: List[str]
    created_at: str

class ChapterService:
    """
    Service class for handling chapter-related operations
    """
    
    def __init__(self, db: Session):
        self.db = db
    
    def create_chapter(self, chapter_data: ChapterCreate) -> Chapter:
        """
        Create a new chapter
        """
        chapter = Chapter(
            module_id=chapter_data.module_id,
            title=chapter_data.title,
            content=chapter_data.content,
            content_variants=str(chapter_data.content_variants),  # Store as JSON string
            order_index=chapter_data.order_index,
            learning_outcomes=chapter_data.learning_outcomes
        )
        self.db.add(chapter)
        self.db.commit()
        self.db.refresh(chapter)
        return chapter
    
    def get_chapter_by_id(self, chapter_id: int) -> Chapter:  # Changed to int
        """
        Retrieve a chapter by its ID
        """
        return self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
    
    def get_all_chapters(self) -> List[Chapter]:
        """
        Retrieve all chapters
        """
        return self.db.query(Chapter).all()
    
    def update_chapter(self, chapter_id: int, chapter_data: ChapterCreate) -> Chapter:  # Changed to int
        """
        Update an existing chapter
        """
        chapter = self.get_chapter_by_id(chapter_id)
        if chapter:
            for key, value in chapter_data.dict().items():
                if key != 'id':  # Don't allow changing ID
                    setattr(chapter, key, value)
            self.db.commit()
            self.db.refresh(chapter)
        return chapter
    
    def delete_chapter(self, chapter_id: int) -> bool:  # Changed to int
        """
        Delete a chapter by its ID
        """
        chapter = self.get_chapter_by_id(chapter_id)
        if chapter:
            # Delete associated content first
            contents = self.db.query(BookContent).filter(
                BookContent.chapter_id == chapter_id
            ).all()
            for content in contents:
                self.db.delete(content)
            
            # Now delete the chapter
            self.db.delete(chapter)
            self.db.commit()
            return True
        return False
    
    def get_full_chapter_content(self, chapter_id: int) -> Optional[Chapter]:  # Changed to int
        """
        Get a chapter with its full content
        """
        chapter = self.get_chapter_by_id(chapter_id)
        return chapter

# Example usage
if __name__ == "__main__":
    print("Chapter service created")