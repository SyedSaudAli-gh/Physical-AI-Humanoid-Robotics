from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from urllib.parse import quote_plus

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:password@localhost:5432/textbook_db")

# Properly encode the URL to handle special characters
encoded_url = quote_plus(DATABASE_URL)

# Create engine with connection pooling
engine = create_engine(
    f"postgresql://{encoded_url}",
    poolclass=QueuePool,
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,
    pool_recycle=300,
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

def get_db():
    """
    Dependency for getting database session
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def create_tables():
    """
    Create all tables in the database
    """
    Base.metadata.create_all(bind=engine)

if __name__ == "__main__":
    create_tables()