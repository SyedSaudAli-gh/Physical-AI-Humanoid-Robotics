import os
import tempfile
from unittest.mock import patch
import pytest
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.src.main import app
from backend.src.database import Base, get_db
from backend.src.models.user import User
from backend.src.auth import auth


@pytest.fixture(scope="session")
def db_engine():
    """Create a temporary in-memory SQLite database for testing."""
    _, tmp_db_path = tempfile.mkstemp(suffix=".db")
    engine = create_engine(f"sqlite:///{tmp_db_path}", connect_args={"check_same_thread": False})
    Base.metadata.create_all(engine)
    yield engine
    os.unlink(tmp_db_path)


@pytest.fixture(scope="function")
def db_session(db_engine):
    """Create a new database session for each test."""
    connection = db_engine.connect()
    transaction = connection.begin()
    SessionLocal = sessionmaker(bind=connection)
    session = SessionLocal()

    yield session

    session.close()
    transaction.rollback()
    connection.close()


@pytest.fixture(scope="function")
def client(db_session):
    """Create a test client with a valid database session."""
    
    def override_get_db():
        yield db_session

    app.dependency_overrides[get_db] = override_get_db
    
    # Also mock the auth dependency to bypass real auth for most tests
    def override_auth(request, optional=False):
        # Return a mock user when needed for authenticated endpoints
        return None
    
    app.dependency_overrides[auth.user] = override_auth

    with patch.dict(os.environ, {
        "BETTER_AUTH_SECRET": "test_secret",
        "COHERE_API_KEY": "test_cohere_key",
        "GEMINI_API_KEY": "test_gemini_key",
        "QDRANT_URL": "http://localhost:6333",
        "QDRANT_API_KEY": "test_qdrant_key"
    }):
        from fastapi.testclient import TestClient
        yield TestClient(app)

    app.dependency_overrides.clear()


@pytest.fixture
def mock_embedding_response():
    """Mock response for Cohere embedding API"""
    return {
        "embeddings": [[0.1, 0.2, 0.3]]
    }


@pytest.fixture
def mock_generation_response():
    """Mock response for Gemini generation API"""
    return {
        "candidates": [
            {
                "content": {
                    "parts": [
                        {"text": "Test response from Gemini"}
                    ]
                }
            }
        ]
    }


@pytest.fixture
def sample_user_data():
    """Sample user data for testing"""
    return {
        "email": "test@example.com",
        "password": "securepassword123",
        "name": "Test User"
    }


@pytest.fixture
def sample_chat_data():
    """Sample chat data for testing"""
    return {
        "message": "Hello, how are you?",
        "selected_text": None
    }


@pytest.fixture
def sample_translation_data():
    """Sample translation data for testing"""
    return {
        "text": "Hello world",
        "target_lang": "es"
    }