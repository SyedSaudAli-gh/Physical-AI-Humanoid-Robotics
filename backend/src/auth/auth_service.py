# Better-Auth configuration for user authentication and profile management
from typing import Optional
from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
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

# Mock database for users (in a real implementation, this would be a DB)
fake_users_db = {}

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password"""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Hash a password"""
    return pwd_context.hash(password)

def get_user(email: str) -> Optional[User]:
    """Get a user from the database by email"""
    user_data = fake_users_db.get(email)
    if user_data:
        return User(**user_data)
    return None

def authenticate_user(email: str, password: str) -> Optional[User]:
    """Authenticate a user with email and password"""
    user = get_user(email)
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

async def get_current_user(token: str = Depends(HTTPBearer())):
    """Get the current user from the token"""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token.credentials, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
        token_data = TokenData(email=email)
    except jwt.PyJWTError:
        raise credentials_exception
    user = get_user(email=token_data.email)
    if user is None:
        raise credentials_exception
    return user

def register_user(user_data: UserRegistration) -> User:
    """Register a new user in the system"""
    if user_data.email in fake_users_db:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    user_id = str(uuid.uuid4())
    hashed_password = get_password_hash(user_data.password)
    
    user = User(
        id=user_id,
        email=user_data.email,
        name=user_data.name,
        technical_skills=user_data.technical_skills,
        experience_level=user_data.experience_level,
        background_questionnaire=user_data.background_questionnaire,
        preferences={}
    )
    
    # Store in mock database (in real app, save to actual DB)
    fake_users_db[user.email] = user.dict()
    fake_users_db[user.email]["hashed_password"] = hashed_password
    
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