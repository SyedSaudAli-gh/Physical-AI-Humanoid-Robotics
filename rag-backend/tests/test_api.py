"""
API endpoint tests with httpx for the RAG Agent application.
"""
import pytest
from fastapi.testclient import TestClient
from main import app
from models import ChatRequest

client = TestClient(app)

def test_chat_endpoint_success():
    """Test that the chat endpoint successfully processes a query."""
    request_data = {
        "query": "What is the Robot Operating System?",
        "selected_text": "ROS"
    }
    
    response = client.post("/api/chat", json=request_data)
    
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert isinstance(data["sources"], list)


def test_chat_endpoint_without_selected_text():
    """Test that the chat endpoint works without selected text."""
    request_data = {
        "query": "Explain machine learning concepts"
    }
    
    response = client.post("/api/chat", json=request_data)
    
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data


def test_chat_endpoint_validation_error():
    """Test that the chat endpoint properly validates input."""
    request_data = {
        "query": ""  # Empty query should fail validation
    }
    
    response = client.post("/api/chat", json=request_data)
    
    assert response.status_code == 422  # Unprocessable Entity


def test_health_endpoint():
    """Test that the health endpoint returns correct status."""
    response = client.get("/health")
    
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["service"] == "RAG Agent API"


def test_root_endpoint():
    """Test that the root endpoint returns correct message."""
    response = client.get("/")
    
    assert response.status_code == 200
    data = response.json()
    assert "message" in data