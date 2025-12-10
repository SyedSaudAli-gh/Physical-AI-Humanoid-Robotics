from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from database import get_db
from models.chapter import Chapter
from services.chapter_service import ChapterService, ChapterCreate
from services.personalization_service import PersonalizationService

router = APIRouter(prefix="/chapters", tags=["chapters"])

@router.get("/{chapter_id}", response_model=dict)
def get_chapter(chapter_id: str,
                difficulty: Optional[str] = None,
                language: Optional[str] = None,
                user_id: Optional[str] = None,  # User ID for personalization
                db: Session = Depends(get_db)):
    """
    Retrieve a specific chapter content
    Optionally apply personalization based on user profile, difficulty and language
    """
    try:
        # If user_id is provided, use personalization service to get tailored content
        if user_id:
            personalization_service = PersonalizationService(db)
            try:
                personalized_content = personalization_service.get_personalized_content(chapter_id, user_id)
                return personalized_content
            except ValueError:
                # If personalization fails (e.g., user not found), fall back to basic approach
                pass

        # Fallback: Use the original approach if no user_id or personalization fails
        chapter_service = ChapterService(db)
        chapter = chapter_service.get_full_chapter_content(chapter_id)

        if not chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )

        # Determine which content to return based on difficulty and language preferences
        content_to_return = chapter.content

        # Apply difficulty-based content selection if content variants exist
        import json
        try:
            content_variants = json.loads(chapter.content_variants) if chapter.content_variants else {}
        except:
            content_variants = {}

        if difficulty and content_variants:
            difficulty_content = content_variants.get(difficulty)
            if difficulty_content:
                content_to_return = difficulty_content

        # Apply language translation if requested
        if language == "ur" and content_variants:
            urdu_content = content_variants.get("urdu")
            if urdu_content:
                content_to_return = urdu_content

        # Return chapter data with content
        result = {
            "id": chapter.id,
            "title": chapter.title,
            "content": content_to_return,
            "module_id": chapter.module_id,
            "order_index": chapter.order_index,
            "learning_outcomes": chapter.learning_outcomes,
            "code_examples": [],  # This would be populated from BookContent model in a full implementation
            "exercises": [],  # This would be populated in a full implementation
            "personalization_applied": False,  # Indicate no personalization was applied in this case
            "message": "No user profile provided, showing default content"
        }

        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving chapter: {str(e)}"
        )