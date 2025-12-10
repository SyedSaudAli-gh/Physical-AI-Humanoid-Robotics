from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    # Database settings
    database_url: str = os.getenv("DATABASE_URL", "postgresql://user:password@localhost:5432/textbook_db")
    
    # Qdrant settings
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    
    # API keys
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    
    # Auth settings
    better_auth_secret: str = os.getenv("BETTER_AUTH_SECRET", "your-default-secret-key-change-in-production")
    better_auth_url: str = os.getenv("BETTER_AUTH_URL", "http://localhost:3000")
    
    # Application settings
    app_name: str = "Physical AI & Humanoid Robotics Textbook API"
    app_version: str = "0.1.0"
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"


settings = Settings()