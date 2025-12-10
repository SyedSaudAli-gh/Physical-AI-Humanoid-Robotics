from typing import Optional
from sqlalchemy.orm import Session
from pydantic import BaseModel
from models.user_profile import UserProfile
from passlib.context import CryptContext
import uuid


pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class UserRegistrationRequest(BaseModel):
    email: str
    password: str
    name: str
    technical_skills: list = []
    experience_level: str = None  # "beginner", "intermediate", "advanced"
    background_questionnaire: dict = {}


class UserService:
    def __init__(self, db: Session):
        self.db = db

    def register_user_with_background(self, user_request: UserRegistrationRequest) -> dict:
        """Register a new user with their background information"""
        
        # Check if user already exists
        existing_user = self.db.query(UserProfile).filter(
            UserProfile.email == user_request.email
        ).first()
        
        if existing_user:
            raise ValueError("User with this email already exists")
        
        # Hash the password
        hashed_password = pwd_context.hash(user_request.password)
        
        # Create new user profile
        user_profile = UserProfile(
            user_id=str(uuid.uuid4()),
            email=user_request.email,
            name=user_request.name,
            technical_skills=user_request.technical_skills,
            experience_level=user_request.experience_level,
            background_questionnaire=user_request.background_questionnaire,
        )
        
        # Additional fields might be added here depending on auth system integration
        # For now, we'll store the password hash in the profile or handle it separately
        # In a real Better-Auth integration, password handling would be abstracted
        
        self.db.add(user_profile)
        self.db.commit()
        self.db.refresh(user_profile)
        
        return user_profile.to_dict()

    def get_user_by_email(self, email: str) -> Optional[dict]:
        """Get user by email"""
        user = self.db.query(UserProfile).filter(UserProfile.email == email).first()
        return user.to_dict() if user else None

    def update_user_background(self, user_id: str, technical_skills: list = None, 
                              experience_level: str = None, background_questionnaire: dict = None) -> dict:
        """Update user background information"""
        user = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
        
        if not user:
            raise ValueError(f"User with ID {user_id} not found")
        
        if technical_skills is not None:
            user.technical_skills = technical_skills
        if experience_level is not None:
            user.experience_level = experience_level
        if background_questionnaire is not None:
            user.background_questionnaire = background_questionnaire
        
        self.db.commit()
        self.db.refresh(user)
        
        return user.to_dict()