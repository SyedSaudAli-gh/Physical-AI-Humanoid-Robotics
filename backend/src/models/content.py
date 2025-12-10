from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class BookContent(Base):
    __tablename__ = "book_content"

    id = Column(String, primary_key=True, index=True)
    chapter_id = Column(String, ForeignKey("chapters.id"), nullable=False)
    content_type = Column(String, nullable=False)  # enum: "text", "code", "diagram", "exercise", "summary"
    content = Column(Text, nullable=False)
    version = Column(Integer, default=1)
    language = Column(String, default="en")
    difficulty_level = Column(String)  # enum: "beginner", "intermediate", "advanced"
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())