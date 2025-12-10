from sqlalchemy import Column, Integer, String, DateTime, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class SubagentConfiguration(Base):
    __tablename__ = "subagent_configurations"

    id = Column(String, primary_key=True, index=True)
    name = Column(String, nullable=False)
    purpose = Column(Text, nullable=False)  # Description of what the subagent does
    parameters = Column(Text)  # JSON parameters for the subagent
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())