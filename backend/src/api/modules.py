from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List
from database import get_db
from models.module import Module
from services.module_service import ModuleService, ModuleCreate

router = APIRouter(prefix="/modules", tags=["modules"])

@router.get("/", response_model=List[dict])
def get_all_modules(db: Session = Depends(get_db)):
    """
    Retrieve all textbook modules
    """
    try:
        module_service = ModuleService(db)
        modules = module_service.get_all_modules()
        
        # Convert modules to response format
        result = []
        for module in modules:
            result.append({
                "id": module.id,
                "name": module.name,
                "description": module.description,
                "order_index": module.order_index,
                "learning_objectives": module.learning_objectives,
                "prerequisites": module.prerequisites or []
            })
        
        return result
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving modules: {str(e)}"
        )


@router.get("/{module_id}/chapters", response_model=List[dict])
def get_module_chapters(module_id: str, db: Session = Depends(get_db)):
    """
    Retrieve chapters for a specific module
    """
    try:
        module_service = ModuleService(db)
        module = module_service.get_module_by_id(module_id)
        
        if not module:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Module not found"
            )
        
        chapters = module_service.get_module_with_chapters(module_id).chapters
        
        # Convert chapters to response format
        result = []
        for chapter in chapters:
            result.append({
                "id": chapter.id,
                "title": chapter.title,
                "order_index": chapter.order_index,
                "learning_outcomes": chapter.learning_outcomes,
                "content_preview": chapter.content[:200] + "..." if len(chapter.content) > 200 else chapter.content
            })
        
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving chapters: {str(e)}"
        )