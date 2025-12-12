# backend/src/auth/auth_service.py
# Better-Auth configuration for user authentication and profile management
from typing import Optional
from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from ..database import get_db
from ..models.user_profile import UserProfile
import jwt
from passlib.context import CryptContext
from datetime import datetime, timedelta
from pydantic import BaseModel
import os
import uuid

# Password hashing setup
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT setup
SECRET_KEY = os.getenv("BETTER_AUTH_SECRET", "your-super-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# User model
class User(BaseModel):
    id: str
    email: str
    name: str
    technical_skills: Optional[list] = []
    experience_level: Optional[str] = None  # "beginner", "intermediate", "advanced"
    background_questionnaire: Optional[dict] = {}
    preferences: Optional[dict] = {}

# User registration model
class UserRegistration(BaseModel):
    email: str
    password: str
    name: str
    technical_skills: Optional[list] = []
    experience_level: Optional[str] = None
    background_questionnaire: Optional[dict] = {}

# User login model
class UserLogin(BaseModel):
    email: str
    password: str

# Token model
class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: str

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password"""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Hash a password"""
    return pwd_context.hash(password)

def get_user(db: Session, email: str) -> Optional[UserProfile]:
    """Get a user from the database by email"""
    user = db.query(UserProfile).filter(UserProfile.email == email).first()
    if user:
        return user
    return None

def authenticate_user(db: Session, email: str, password: str) -> Optional[UserProfile]:
    """Authenticate a user with email and password"""
    user = get_user(db, email)
    if not user or not verify_password(password, user.hashed_password):
        return None
    return user

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token"""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

from fastapi import Depends
from sqlalchemy.orm import Session

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer()),
    db: Session = Depends(get_db)  # Fixed: Add this dependency
):
    """Get the current user from the token"""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    token = credentials.credentials

    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
        token_data = TokenData(email=email)
    except jwt.PyJWTError:
        raise credentials_exception

    user = get_user(db, email=token_data.email)  # Now using the db from Depends
    if user is None:
        raise credentials_exception
    return user

def register_user(db: Session, user_data: UserRegistration) -> UserProfile:
    """Register a new user in the system"""
    # Check if user already exists
    existing_user = db.query(UserProfile).filter(UserProfile.email == user_data.email).first()
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")

    # Hash the password
    hashed_password = get_password_hash(user_data.password)

    # Create user profile in database
    user = UserProfile(
        email=user_data.email,
        name=user_data.name,
        technical_skills=user_data.technical_skills or [],
        experience_level=user_data.experience_level,
        background_questionnaire=user_data.background_questionnaire or {},
        hashed_password=hashed_password  # Store the hashed password in the UserProfile
    )

    # Add to database
    db.add(user)
    db.commit()
    db.refresh(user)

    return user

# In a real implementation, this would be integrated into your FastAPI app
# app = FastAPI()
#
# @app.post("/auth/register", response_model=User)
# async def register(user_data: UserRegistration):
#     return register_user(user_data)
#
# @app.post("/auth/login", response_model=Token)
# async def login(user_data: UserLogin):
#     user = authenticate_user(user_data.email, user_data.password)
#     if not user:
#         raise HTTPException(status_code=400, detail="Incorrect email or password")
#     
#     access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
#     access_token = create_access_token(
#         data={"sub": user.email}, expires_delta=access_token_expires
#     )
#     
#     return {"access_token": access_token, "token_type": "bearer"}

if __name__ == "__main__":
    print("Better-Auth configuration for user authentication and profile management")
    print("This module provides user registration, login, and authentication services.")