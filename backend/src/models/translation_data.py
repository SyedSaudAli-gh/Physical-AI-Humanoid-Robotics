from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class TranslationData(Base):
    __tablename__ = "translation_data"

    id = Column(String, primary_key=True, index=True)
    source_content_id = Column(String, ForeignKey("book_content.id"), nullable=False)
    target_language = Column(String, nullable=False)
    translated_content = Column(Text, nullable=False)
    verification_status = Column(String, default="pending")  # enum: "pending", "verified", "needs_revision"
    verified_by = Column(String)  # Who verified the translation
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())