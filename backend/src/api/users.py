from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Any
from pydantic import BaseModel
from ..database import get_db
from ..services.user_service import UserService

router = APIRouter(prefix="/users", tags=["users"])

# Request model for updating user preferences
class UpdatePreferencesRequest(BaseModel):
    preferred_language: str = None
    chapter_difficulty_override: str = None  # "beginner", "intermediate", "advanced"
    notification_preferences: dict = None

# Response model for preferences update
class UpdatePreferencesResponse(BaseModel):
    user_id: str
    preferences: dict
    updated_at: str

@router.put("/preferences", response_model=UpdatePreferencesResponse)
def update_user_preferences(
    request: UpdatePreferencesRequest, 
    db: Session = Depends(get_db),
    # In a real implementation, you'd get the user ID from authentication
    # For now, using a placeholder
    current_user_id: str = "placeholder_user_id"
):
    """
    Update user preferences including language and content difficulty
    """
    try:
        user_service = UserService(db)
        
        # Prepare preferences to update
        preferences_to_update = {}
        if request.preferred_language is not None:
            preferences_to_update['preferred_language'] = request.preferred_language
        if request.chapter_difficulty_override is not None:
            preferences_to_update['chapter_difficulty_override'] = request.chapter_difficulty_override
        if request.notification_preferences is not None:
            if 'notification_preferences' in preferences_to_update:
                preferences_to_update['notification_preferences'].update(request.notification_preferences)
            else:
                preferences_to_update['notification_preferences'] = request.notification_preferences
        
        # Update user preferences
        success = user_service.update_user_preferences(current_user_id, preferences_to_update)
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )
        
        # Fetch updated user to return preferences
        updated_user = user_service.get_user_by_id(current_user_id)
        
        return UpdatePreferencesResponse(
            user_id=updated_user.id,
            preferences=updated_user.preferences or {},
            updated_at=updated_user.updated_at.isoformat() if updated_user.updated_at else None
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating preferences: {str(e)}"
        )