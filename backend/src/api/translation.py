from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional
from ..database import get_db
from ..services.translation_service import TranslationService

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
        
        # For this endpoint, we'll use the source_content_id as a placeholder
        # since we're translating arbitrary content
        source_content_id = "adhoc_content"  # Placeholder for ad-hoc translations
        
        translated_content = translation_service.translate_content(
            request.content, 
            request.target_language, 
            request.source_language
        )
        
        # In a real implementation, we would store the translation
        # For now, we'll just return the translation
        # translation_service.store_translation(source_content_id, request.target_language, translated_content)
        
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