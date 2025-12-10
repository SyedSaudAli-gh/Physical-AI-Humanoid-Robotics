from sqlalchemy import Column, Integer, String, DateTime, Text, ARRAY
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class Module(Base):
    __tablename__ = "modules"

    id = Column(String, primary_key=True, index=True)
    name = Column(String, nullable=False)
    description = Column(Text, nullable=False)
    order_index = Column(Integer, nullable=False)
    learning_objectives = Column(ARRAY(String), nullable=False)
    prerequisites = Column(ARRAY(String))
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())