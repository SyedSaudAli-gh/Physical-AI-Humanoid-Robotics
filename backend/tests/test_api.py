"""
Basic API tests for the Physical AI & Humanoid Robotics Textbook project
"""
import pytest
from fastapi.testclient import TestClient
from src.main import app

client = TestClient(app)

def test_read_root():
    """Test the root endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()

def test_health_endpoint():
    """Test the health endpoint."""
    response = client.get("/health") 
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}

# Additional API tests would go here as needed