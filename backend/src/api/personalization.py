from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.orm import Session
from typing import Dict, Any
from pydantic import BaseModel
from ..database import get_db
from ..services.personalization_service import PersonalizationService


router = APIRouter(prefix="/personalize", tags=["personalization"])

class PersonalizeRequest(BaseModel):
    chapter_id: int
    user_id: str

class PersonalizeResponse(BaseModel):
    id: int
    title: str
    content: str
    original_content: str
    user_experience_level: str = None
    adapted_for_skills: list = []

@router.post("/", response_model=PersonalizeResponse)
def personalize_chapter(request: PersonalizeRequest, db: Session = Depends(get_db)):
    """
    Personalize chapter content based on user's background
    """
    try:
        service = PersonalizationService(db)
        result = service.get_personalized_chapter_content(request.chapter_id, request.user_id)

        return PersonalizeResponse(
            id=result["id"],
            title=result["title"],
            content=result["content"],
            original_content=result["original_content"],
            user_experience_level=result["user_experience_level"],
            adapted_for_skills=result["adapted_for_skills"]
        )
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")

@router.post("/chapter", response_model=PersonalizeResponse)
def get_personalized_chapter(request: PersonalizeRequest, db: Session = Depends(get_db)):
    """
    Alternative endpoint for personalizing chapter content
    """
    try:
        service = PersonalizationService(db)
        result = service.get_personalized_chapter_content(request.chapter_id, request.user_id)

        return PersonalizeResponse(
            id=result["id"],
            title=result["title"],
            content=result["content"],
            original_content=result["original_content"],
            user_experience_level=result["user_experience_level"],
            adapted_for_skills=result["adapted_for_skills"]
        )
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")