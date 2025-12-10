from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from urllib.parse import quote_plus

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./textbook.db")

# Handle different database URLs appropriately
if DATABASE_URL.startswith("postgresql"):
    # Properly encode the URL to handle special characters for PostgreSQL
    encoded_url = quote_plus(DATABASE_URL)
    engine = create_engine(
        f"postgresql://{encoded_url}",
        poolclass=QueuePool,
        pool_size=10,
        max_overflow=20,
        pool_pre_ping=True,
        pool_recycle=300,
    )
else:
    # For SQLite (default for development)
    engine = create_engine(
        DATABASE_URL,
        connect_args={"check_same_thread": False}  # Needed for SQLite
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