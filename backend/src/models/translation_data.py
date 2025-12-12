from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.sql import func
from ..database import Base


class TranslationData(Base):
    __tablename__ = "translation_data"

    id = Column(Integer, primary_key=True, index=True)
    source_content_id = Column(String, nullable=False)  # Could be chapter_1, content_23, etc.
    target_language = Column(String, nullable=False)
    source_language = Column(String, default="en")
    translated_content = Column(Text, nullable=False)
    verification_status = Column(String, default="pending")  # enum: "pending", "verified", "needs_revision"
    verified_by = Column(String)  # Who verified the translation
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def to_dict(self):
        return {
            "id": self.id,
            "source_content_id": self.source_content_id,
            "target_language": self.target_language,
            "source_language": self.source_language,
            "translated_content": self.translated_content,
            "verification_status": self.verification_status,
            "verified_by": self.verified_by,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None
        }