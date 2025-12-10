from sqlalchemy import Column, Integer, String, DateTime, Text, ARRAY, ForeignKey
from sqlalchemy.dialects.postgresql import ARRAY as PG_ARRAY
from sqlalchemy.types import Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class RagIndex(Base):
    __tablename__ = "rag_index"

    id = Column(String, primary_key=True, index=True)
    content_id = Column(String, ForeignKey("book_content.id"), nullable=False)
    embedding = Column(PG_ARRAY(Float))  # Array of numbers for the embedding
    chunk_text = Column(Text, nullable=False)
    chunk_metadata = Column(Text)  # JSON: chapter_id, module_id, content_type, difficulty_level
    created_at = Column(DateTime(timezone=True), server_default=func.now())