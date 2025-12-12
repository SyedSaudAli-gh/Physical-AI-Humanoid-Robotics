from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
from datetime import timedelta
from ..database import get_db
from sqlalchemy.orm import Session
from ..auth.auth_service import (
    UserLogin,
    Token,
    get_user,
    authenticate_user,
    create_access_token,
    ACCESS_TOKEN_EXPIRE_MINUTES
)

# Import the signup router
from ..auth.signup import router as signup_router

# Main auth router that includes all auth-related endpoints
auth_router = APIRouter()

# Include signup router
auth_router.include_router(signup_router)

# Add login endpoint
@auth_router.post("/login", response_model=Token)
async def login(user_data: UserLogin, db: Session = Depends(get_db)):
    """Authenticate user and return access token"""
    user = authenticate_user(db, user_data.email, user_data.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Incorrect email or password"
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

# Endpoint to get current user info
security = HTTPBearer()

@auth_router.get("/me")
async def read_users_me(credentials: HTTPAuthorizationCredentials = Depends(security), db: Session = Depends(get_db)):
    """Get current user info based on token"""
    from auth.auth_service import get_current_user
    user = await get_current_user(credentials)
    # Return user data without sensitive info like password
    user_data = user.__dict__.copy()
    if 'hashed_password' in user_data:
        del user_data['hashed_password']
    return {"user": user_data}

__all__ = ["auth_router"]