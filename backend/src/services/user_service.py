from sqlalchemy.orm import Session
from typing import Dict, Any, Optional
from ..models.user_profile import User
from ..auth.auth_service import register_user, UserRegistration
from pydantic import BaseModel
from datetime import datetime
import uuid

class UserRegistrationRequest(BaseModel):
    email: str
    password: str
    name: str
    technical_skills: Optional[list] = []
    experience_level: Optional[str] = None  # "beginner", "intermediate", "advanced"
    background_questionnaire: Optional[dict] = {}

class UserService:
    """
    Service class for handling user-related operations including registration with background information
    """
    
    def __init__(self, db: Session):
        self.db = db
    
    def register_user_with_background(self, user_data: UserRegistrationRequest) -> Dict[str, Any]:
        """
        Register a new user with technical background information
        """
        # Prepare data for the auth service
        auth_user_data = UserRegistration(
            email=user_data.email,
            password=user_data.password,
            name=user_data.name,
            technical_skills=user_data.technical_skills,
            experience_level=user_data.experience_level,
            background_questionnaire=user_data.background_questionnaire
        )
        
        # Use the existing auth service to register the user
        user = register_user(auth_user_data)
        
        # Update the user in the database with additional information
        db_user = User(
            id=user.id,
            email=user.email,
            name=user.name,
            hashed_password=user.hashed_password,  # This would come from the auth system
            technical_skills=user.technical_skills,
            experience_level=user.experience_level,
            background_questionnaire=user.background_questionnaire,
            preferences=user.preferences,
            created_at=datetime.now(),
            updated_at=datetime.now()
        )
        
        self.db.add(db_user)
        self.db.commit()
        self.db.refresh(db_user)
        
        # Return user data without sensitive information
        return {
            'id': db_user.id,
            'email': db_user.email,
            'name': db_user.name,
            'technical_skills': db_user.technical_skills,
            'experience_level': db_user.experience_level,
            'background_questionnaire': db_user.background_questionnaire,
            'preferences': db_user.preferences
        }
    
    def get_user_by_id(self, user_id: str) -> Optional[User]:
        """
        Retrieve user by ID
        """
        return self.db.query(User).filter(User.id == user_id).first()
    
    def update_user_background(self, user_id: str, technical_skills: list = None, 
                              experience_level: str = None, background_questionnaire: dict = None) -> bool:
        """
        Update user's background information
        """
        user = self.get_user_by_id(user_id)
        if not user:
            return False
        
        if technical_skills is not None:
            user.technical_skills = technical_skills
        if experience_level is not None:
            user.experience_level = experience_level
        if background_questionnaire is not None:
            user.background_questionnaire = background_questionnaire
        
        user.updated_at = datetime.now()
        self.db.commit()
        return True
    
    def update_user_preferences(self, user_id: str, preferences: dict) -> bool:
        """
        Update user preferences
        """
        user = self.get_user_by_id(user_id)
        if not user:
            return False
        
        # Merge new preferences with existing ones
        if user.preferences:
            user.preferences.update(preferences)
        else:
            user.preferences = preferences
        
        user.updated_at = datetime.now()
        self.db.commit()
        return True

# Example usage
if __name__ == "__main__":
    print("User service created")