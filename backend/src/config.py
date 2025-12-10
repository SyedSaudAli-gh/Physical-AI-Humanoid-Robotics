# Environment configuration management with quality gates
from pydantic import BaseSettings, validator
from typing import Optional, List
import os

class Settings(BaseSettings):
    """
    Application settings with validation
    """
    # Database settings
    database_url: str = "postgresql://user:password@localhost:5432/textbook_db"
    
    # API Keys and External Services
    cohere_api_key: Optional[str] = None
    openai_api_key: Optional[str] = None
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    
    # Authentication
    better_auth_secret: Optional[str] = None
    better_auth_url: str = "http://localhost:3000"
    
    # Application settings
    app_name: str = "Physical AI & Humanoid Robotics Textbook"
    app_version: str = "0.1.0"
    environment: str = "development"  # development, staging, production
    
    # Quality gates and thresholds
    max_content_similarity_threshold: float = 0.8  # For plagiarism detection
    min_readability_score: float = 10.0  # Flesch-Kincaid grade level
    max_response_time: float = 3.0  # Maximum allowed response time in seconds
    min_content_length: int = 2000  # Minimum content length for chapters
    
    # List of required environment variables based on environment
    required_env_vars: List[str] = []
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
    
    @validator('cohere_api_key')
    def validate_cohere_api_key(cls, v):
        if v is None:
            # In development, we might allow this, but in production it's required
            if os.getenv('environment', 'development') == 'production':
                raise ValueError('COHERE_API_KEY is required in production environment')
        return v
    
    @validator('qdrant_url')
    def validate_qdrant_url(cls, v):
        if v is None:
            if os.getenv('environment', 'development') == 'production':
                raise ValueError('QDRANT_URL is required in production environment')
        return v
    
    @validator('better_auth_secret')
    def validate_auth_secret(cls, v):
        if v is None:
            if os.getenv('environment', 'development') == 'production':
                raise ValueError('BETTER_AUTH_SECRET is required in production environment')
        elif len(v) < 32:
            raise ValueError('BETTER_AUTH_SECRET should be at least 32 characters long')
        return v
    
    def is_production(self) -> bool:
        return self.environment.lower() == 'production'
    
    def is_development(self) -> bool:
        return self.environment.lower() == 'development'


def validate_environment(settings: Settings) -> List[str]:
    """
    Validate environment configuration against quality gates
    """
    issues = []
    
    # Check if running in production without required keys
    if settings.is_production():
        if not settings.cohere_api_key:
            issues.append("Production environment requires COHERE_API_KEY")
        if not settings.qdrant_url:
            issues.append("Production environment requires QDRANT_URL")
        if not settings.better_auth_secret or len(settings.better_auth_secret) < 32:
            issues.append("Production environment requires BETTER_AUTH_SECRET of at least 32 characters")
    
    # Validate API key formats (basic validation)
    if settings.cohere_api_key and len(settings.cohere_api_key) < 10:
        issues.append("COHERE_API_KEY appears to be too short")
    
    # Validate database URL
    if not settings.database_url or "postgresql" not in settings.database_url.lower():
        issues.append("DATABASE_URL should be a valid PostgreSQL connection string")
    
    # Validate quality thresholds
    if settings.max_content_similarity_threshold < 0.5 or settings.max_content_similarity_threshold > 0.95:
        issues.append("MAX_CONTENT_SIMILARITY_THRESHOLD should be between 0.5 and 0.95")
    
    if settings.min_readability_score < 0 or settings.min_readability_score > 15:
        issues.append("MIN_READABILITY_SCORE should be between 0 and 15")
    
    return issues


# Create a global settings instance
settings = Settings()

def setup_environment():
    """
    Setup environment with quality gates
    """
    print(f"Environment: {settings.environment}")
    print(f"App: {settings.app_name} v{settings.app_version}")
    
    issues = validate_environment(settings)
    
    if issues:
        print("Environment validation issues found:")
        for issue in issues:
            print(f"  - {issue}")
        return False
    
    print("Environment validation passed")
    return True


if __name__ == "__main__":
    if setup_environment():
        print("Environment is properly configured with quality gates in place")
    else:
        print("Environment configuration has issues that need to be addressed")