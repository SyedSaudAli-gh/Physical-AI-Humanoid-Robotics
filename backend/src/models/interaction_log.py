from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class InteractionLog(Base):
    __tablename__ = "interaction_logs"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=False)
    interaction_type = Column(String, nullable=False)  # enum: "chat_query", "chapter_view", "translation", "personalization", "exercise", "code_run"
    content_id = Column(String, ForeignKey("book_content.id"))  # Optional reference
    query_text = Column(Text)  # For RAG queries
    response_text = Column(Text)  # For RAG responses
    selected_text = Column(Text)  # For text selected by user
    personalization_applied = Column(Text)  # JSON: original_difficulty, applied_difficulty, content_changed
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    session_id = Column(String, nullable=False)