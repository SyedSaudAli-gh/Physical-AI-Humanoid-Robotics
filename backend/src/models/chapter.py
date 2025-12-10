from sqlalchemy import Column, Integer, String, ForeignKey, Text, DateTime, Boolean
from sqlalchemy.sql import func
from database import Base


class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, nullable=False)
    module_id = Column(Integer, ForeignKey("modules.id"))
    content = Column(Text)
    content_variants = Column(Text)  # JSON string containing different difficulty levels
    urdu_translation = Column(Text)  # Translated content in Urdu
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    is_active = Column(Boolean, default=True)

    def to_dict(self):
        return {
            "id": self.id,
            "title": self.title,
            "module_id": self.module_id,
            "content": self.content,
            "content_variants": self.content_variants,
            "urdu_translation": self.urdu_translation,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
            "is_active": self.is_active
        }