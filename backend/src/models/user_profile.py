from sqlalchemy import Column, Integer, String, DateTime, Text, ARRAY, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    name = Column(String, nullable=False)
    hashed_password = Column(String, nullable=False)
    technical_skills = Column(ARRAY(String))
    experience_level = Column(String)  # enum: "beginner", "intermediate", "advanced"
    background_questionnaire = Column(JSON)  # Object with years_experience, primary_language, etc.
    preferences = Column(JSON)  # Object with preferred_language, chapter_difficulty_override, etc.
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())