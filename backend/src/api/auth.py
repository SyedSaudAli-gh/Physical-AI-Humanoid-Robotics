from fastapi import APIRouter
from ..auth.signup import router as signup_router

# Main auth router that includes all auth-related endpoints
auth_router = APIRouter()

# Include signup router
auth_router.include_router(signup_router)

__all__ = ["auth_router"]