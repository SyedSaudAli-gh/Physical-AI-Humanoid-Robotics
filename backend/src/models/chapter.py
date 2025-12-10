from sqlalchemy import Column, Integer, String, DateTime, Text, ARRAY, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(String, primary_key=True, index=True)
    module_id = Column(String, ForeignKey("modules.id"), nullable=False)
    title = Column(String, nullable=False)
    content = Column(Text, nullable=False)  # Main content (2000-4000 words)
    content_variants = Column(Text)  # JSON: beginner, intermediate, advanced, urdu
    order_index = Column(Integer, nullable=False)
    learning_outcomes = Column(ARRAY(String), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())