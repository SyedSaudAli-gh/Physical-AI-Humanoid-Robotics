"""
Security hardening for the Physical AI & Humanoid Robotics Textbook project

This module implements security measures for authentication and API endpoints
"""
from fastapi import FastAPI, HTTPException, status, Depends, Request
from fastapi.security import HTTPBearer
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.middleware.httpsredirect import HTTPSRedirectMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from typing import Optional, Dict, Any
import re
import logging
from passlib.context import CryptContext
from jose import JWTError, jwt
import os
from datetime import datetime, timedelta

# Configure security-related logging
logging.basicConfig(level=logging.INFO)
security_logger = logging.getLogger("security")

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT settings
SECRET_KEY = os.getenv("BETTER_AUTH_SECRET", "your-default-secret-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# Security headers middleware
async def add_security_headers(request: Request, call_next):
    """
    Add security headers to responses
    """
    response = await call_next(request)
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
    response.headers["Referrer-Policy"] = "no-referrer-when-downgrade"
    return response

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password
    """
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """
    Hash a password
    """
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """
    Create a JWT access token
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def decode_token(token: str):
    """
    Decode and verify a JWT token
    """
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None

def get_current_user_id(token: str = Depends(HTTPBearer())):
    """
    Get current user ID from the token
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token.credentials, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise credentials_exception
        return user_id
    except JWTError:
        raise credentials_exception

class InputValidator:
    """
    Class for validating and sanitizing user inputs
    """
    
    @staticmethod
    def sanitize_input(user_input: str) -> str:
        """
        Sanitize user input to prevent injection attacks
        """
        if not isinstance(user_input, str):
            raise ValueError("Input must be a string")
        
        # Remove potentially dangerous characters
        sanitized = user_input.replace('<script>', '').replace('</script>', '')
        sanitized = sanitized.replace('<iframe>', '').replace('</iframe>', '')
        sanitized = sanitized.replace('<object>', '').replace('</object>', '')
        sanitized = sanitized.replace('javascript:', '')
        sanitized = sanitized.replace('vbscript:', '')
        
        return sanitized
    
    @staticmethod
    def validate_email(email: str) -> bool:
        """
        Validate email format
        """
        pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return re.match(pattern, email) is not None
    
    @staticmethod
    def validate_password_strength(password: str) -> tuple[bool, str]:
        """
        Validate password strength
        Returns (is_strong, message)
        """
        if len(password) < 8:
            return False, "Password must be at least 8 characters long"
        
        if not re.search(r'[A-Z]', password):
            return False, "Password must contain at least one uppercase letter"
        
        if not re.search(r'[a-z]', password):
            return False, "Password must contain at least one lowercase letter"
        
        if not re.search(r'[0-9]', password):
            return False, "Password must contain at least one number"
        
        if not re.search(r'[!@#$%^&*(),.?":{}|<>]', password):
            return False, "Password must contain at least one special character"
        
        return True, "Password is strong enough"

class SecurityMiddleware:
    """
    Security middleware to protect against common vulnerabilities
    """
    
    @staticmethod
    def install_security_middleware(app: FastAPI):
        """
        Install security middleware to the FastAPI app
        """
        # Add rate limiting
        app.state.limiter = limiter
        app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)
        
        # Add security headers
        app.middleware('http')(add_security_headers)
        
        # Add trusted host middleware
        app.add_middleware(
            TrustedHostMiddleware, 
            allowed_hosts=os.getenv("ALLOWED_HOSTS", "localhost,127.0.0.1").split(",")
        )
        
        # In production, you'd also add HTTPS redirect middleware if behind a proxy
        # app.add_middleware(HTTPSRedirectMiddleware)

class APISecurity:
    """
    Security functions for API endpoints
    """
    
    @staticmethod
    @limiter.limit("100/minute")  # Rate limit to 100 requests per minute per IP
    def secured_endpoint(request: Request):
        """
        Example of a rate-limited endpoint
        """
        pass
    
    @staticmethod
    def log_security_event(event_type: str, details: Dict[str, Any], request: Request = None):
        """
        Log security-related events
        """
        event_details = {
            "event_type": event_type,
            "timestamp": datetime.utcnow().isoformat(),
            "details": details
        }
        
        if request:
            event_details["ip_address"] = request.client.host
            event_details["user_agent"] = request.headers.get("user-agent", "unknown")
        
        security_logger.info(f"Security Event: {event_details}")

def setup_security_headers(app: FastAPI):
    """
    Apply security headers and measures to the FastAPI app
    """
    # Install security middleware
    SecurityMiddleware.install_security_middleware(app)
    
    # Add security-related exception handlers
    @app.exception_handler(RateLimitExceeded)
    async def rate_limit_handler(request: Request, exc: RateLimitExceeded):
        APISecurity.log_security_event(
            "RATE_LIMIT_EXCEEDED", 
            {"rate_limit": str(exc.detail)}, 
            request
        )
        return HTTPException(
            status_code=429,
            detail="Rate limit exceeded: slow down!"
        )

# Example usage in API endpoints
def authenticate_user(email: str, password: str):
    """
    Authenticate user credentials
    """
    # This is a simplified example - in a real application, you'd query your database
    # and properly verify the username/password combination
    
    # Log authentication attempt
    APISecurity.log_security_event(
        "AUTHENTICATION_ATTEMPT", 
        {"email": email, "success": False}  # Would be True if successful
    )
    
    # Validate inputs
    if not InputValidator.validate_email(email):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid email format"
        )
    
    # Verify password strength if registering
    is_strong, message = InputValidator.validate_password_strength(password)
    if not is_strong:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=message
        )
    
    # Sanitize inputs
    sanitized_email = InputValidator.sanitize_input(email)
    
    # In a real app, you'd look up the user in the database
    # and verify the password
    return None  # Placeholder

if __name__ == "__main__":
    print("Security hardening utilities loaded")
    print("These include:")
    print("- Authentication and authorization")
    print("- Input validation and sanitization")
    print("- Rate limiting")
    print("- Security headers")
    print("- Security logging")