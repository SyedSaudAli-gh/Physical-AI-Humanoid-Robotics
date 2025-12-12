# backend/src/database.py
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from urllib.parse import urlparse, quote_plus, ParseResult

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./textbook.db")

# Function to properly quote only the password in PostgreSQL URL
def quote_password_in_url(url: str) -> str:
    parsed = urlparse(url)
    if parsed.scheme.startswith('postgres'):
        # Split netloc into user:pass@host:port
        netloc_parts = parsed.netloc.split('@')
        if len(netloc_parts) == 2:
            user_pass = netloc_parts[0].split(':')
            if len(user_pass) == 2:
                user = user_pass[0]
                password = quote_plus(user_pass[1])
                netloc = f"{user}:{password}@{netloc_parts[1]}"
                parsed = ParseResult(
                    scheme=parsed.scheme,
                    netloc=netloc,
                    path=parsed.path,
                    params=parsed.params,
                    query=parsed.query,
                    fragment=parsed.fragment
                )
    return parsed.geturl()

# Handle different database URLs appropriately
if DATABASE_URL.startswith("postgresql"):
    # Properly quote the password
    quoted_url = quote_password_in_url(DATABASE_URL)
    engine = create_engine(
        quoted_url,
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