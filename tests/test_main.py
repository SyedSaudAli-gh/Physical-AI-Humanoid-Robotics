import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import json


def test_root_endpoint(client):
    """Test the root endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    assert "Welcome to the Physical AI Humanoid Robotics API" in response.json()["message"]


def test_health_check(client):
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"


def test_chat_endpoint_success(client, sample_chat_data):
    """Test the chat endpoint with valid input"""
    with patch('backend.src.main.generate_response') as mock_generate:
        mock_generate.return_value = "Test response from AI"
        
        response = client.post("/api/chat", json=sample_chat_data)
        
        assert response.status_code == 200
        assert "response" in response.json()
        assert response.json()["response"] == "Test response from AI"


def test_chat_endpoint_with_selected_text(client):
    """Test the chat endpoint with selected text"""
    payload = {
        "message": "Summarize this",
        "selected_text": "This is the selected text that needs summarization."
    }
    
    with patch('backend.src.main.generate_response') as mock_generate:
        mock_generate.return_value = "Summary: Selected text summarized."
        
        response = client.post("/api/chat", json=payload)
        
        assert response.status_code == 200
        assert "response" in response.json()


def test_chat_endpoint_missing_message(client):
    """Test the chat endpoint with missing message"""
    response = client.post("/api/chat", json={})
    assert response.status_code == 422  # Validation error


def test_translate_endpoint_success(client, sample_translation_data):
    """Test the translate endpoint with valid input"""
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.return_value = "Hola mundo"
        
        response = client.post("/api/translate", json=sample_translation_data)
        
        assert response.status_code == 200
        assert "translated_text" in response.json()
        assert response.json()["translated_text"] == "Hola mundo"


def test_translate_endpoint_missing_params(client):
    """Test the translate endpoint with missing parameters"""
    response = client.post("/api/translate", json={"text": "Hello"})
    assert response.status_code == 422  # Validation error


def test_chat_endpoint_error_handling(client, sample_chat_data):
    """Test the chat endpoint error handling"""
    with patch('backend.src.main.generate_response') as mock_generate:
        mock_generate.side_effect = Exception("API Error")
        
        response = client.post("/api/chat", json=sample_chat_data)
        
        assert response.status_code == 500
        assert "error" in response.json()


def test_translate_endpoint_error_handling(client, sample_translation_data):
    """Test the translate endpoint error handling"""
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.side_effect = Exception("Translation Error")
        
        response = client.post("/api/translate", json=sample_translation_data)
        
        assert response.status_code == 500
        assert "error" in response.json()


def test_nonexistent_endpoint(client):
    """Test a nonexistent endpoint"""
    response = client.get("/nonexistent")
    assert response.status_code == 404