from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional
from database import get_db
from services.translation_service import TranslationService

router = APIRouter(prefix="/translate", tags=["translation"])

# Request model for translation
class TranslationRequest(BaseModel):
    content: str
    target_language: str
    source_language: Optional[str] = "en"

# Response model for translation
class TranslationResponse(BaseModel):
    original_content: str
    translated_content: str
    target_language: str
    source_language: str

@router.post("/", response_model=TranslationResponse)
def translate_content(
    request: TranslationRequest,
    db: Session = Depends(get_db)
):
    """
    Translate content to specified language
    """
    try:
        translation_service = TranslationService(db)

        translated_content = translation_service.translate_content(
            request.content,
            request.target_language,
            request.source_language
        )

        return TranslationResponse(
            original_content=request.content,
            translated_content=translated_content,
            target_language=request.target_language,
            source_language=request.source_language
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}"
        )

# Additional functionality for translating specific content items
class ContentTranslationRequest(BaseModel):
    source_content_id: str
    target_language: str

@router.post("/content", response_model=TranslationResponse)
def translate_existing_content(
    request: ContentTranslationRequest,
    db: Session = Depends(get_db)
):
    """
    Translate existing content by ID
    """
    try:
        translation_service = TranslationService(db)

        # In a real implementation, we would retrieve the content by ID
        # For now, we'll simulate with placeholder content
        # content = get_content_by_id(request.source_content_id)
        content = f"Placeholder content for ID: {request.source_content_id}"

        translation = translation_service.get_or_create_translation(
            request.source_content_id,
            content,
            request.target_language
        )

        return TranslationResponse(
            original_content=content,
            translated_content=translation,
            target_language=request.target_language,
            source_language="en"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}"
        )

# Additional endpoint specifically for chapter translation
class ChapterTranslationRequest(BaseModel):
    chapter_id: int

class ChapterTranslationResponse(BaseModel):
    id: int
    title: str
    urdu_content: str

@router.post("/chapter-urdu", response_model=ChapterTranslationResponse)
def translate_chapter_to_urdu(
    request: ChapterTranslationRequest,
    db: Session = Depends(get_db)
):
    """
    Translate a chapter specifically to Urdu
    """
    try:
        translation_service = TranslationService(db)
        result = translation_service.translate_chapter_to_urdu(request.chapter_id)

        return ChapterTranslationResponse(
            id=result["id"],
            title=result["title"],
            urdu_content=result["urdu_content"]
        )
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Chapter translation failed: {str(e)}"
        )