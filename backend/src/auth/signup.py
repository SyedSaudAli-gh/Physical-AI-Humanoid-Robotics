# Secure signup functionality via Better-Auth
# This file extends the existing auth_service with signup specific functionality
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Any
from pydantic import BaseModel
from database import get_db
from services.user_service import UserService, UserRegistrationRequest

router = APIRouter(prefix="/auth", tags=["auth"])

# Request model for user signup
class SignupRequest(BaseModel):
    email: str
    password: str
    name: str
    technical_skills: list = []
    experience_level: str = None  # "beginner", "intermediate", "advanced"
    background_questionnaire: dict = {}

# Response model for signup
class SignupResponse(BaseModel):
    id: str
    email: str
    name: str
    technical_skills: list
    experience_level: str = None
    background_questionnaire: dict = {}
    message: str

@router.post("/signup", response_model=SignupResponse)
def signup(signup_data: SignupRequest, db: Session = Depends(get_db)):
    """
    Secure user signup with background information collection
    """
    try:
        # Prepare user registration request
        user_request = UserRegistrationRequest(
            email=signup_data.email,
            password=signup_data.password,
            name=signup_data.name,
            technical_skills=signup_data.technical_skills,
            experience_level=signup_data.experience_level,
            background_questionnaire=signup_data.background_questionnaire
        )
        
        # Register user with background information
        user_service = UserService(db)
        user = user_service.register_user_with_background(user_request)
        
        return SignupResponse(
            id=user['id'],
            email=user['email'],
            name=user['name'],
            technical_skills=user['technical_skills'],
            experience_level=user['experience_level'],
            background_questionnaire=user['background_questionnaire'],
            message="User registered successfully"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Registration failed: {str(e)}"
        )

# Include this router in the main auth API