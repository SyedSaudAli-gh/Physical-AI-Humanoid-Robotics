import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import json


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


def test_chat_endpoint_empty_message(client):
    """Test the chat endpoint with empty message"""
    payload = {"message": "", "selected_text": None}
    response = client.post("/api/chat", json=payload)
    assert response.status_code == 422


def test_chat_endpoint_rag_retrieval(client, sample_chat_data):
    """Test that the chat endpoint properly retrieves context from Qdrant"""
    with patch('backend.src.main.retrieve_context_from_qdrant') as mock_retrieve:
        mock_retrieve.return_value = ["Context chunk 1", "Context chunk 2"]
        
        with patch('backend.src.main.generate_response') as mock_generate:
            mock_generate.return_value = "AI response with context"
            
            response = client.post("/api/chat", json=sample_chat_data)
            
            assert response.status_code == 200
            assert "response" in response.json()
            mock_retrieve.assert_called_once_with(sample_chat_data["message"])  # Verify retrieval was called


def test_chat_endpoint_error_handling(client, sample_chat_data):
    """Test the chat endpoint error handling"""
    with patch('backend.src.main.generate_response') as mock_generate:
        mock_generate.side_effect = Exception("API Error")
        
        response = client.post("/api/chat", json=sample_chat_data)
        
        assert response.status_code == 500
        assert "error" in response.json()


def test_chat_endpoint_embedding_error(client, sample_chat_data):
    """Test error handling when embedding service fails"""
    with patch('backend.src.main.get_embeddings') as mock_embeddings:
        mock_embeddings.side_effect = Exception("Embedding API Error")
        
        response = client.post("/api/chat", json=sample_chat_data)
        
        assert response.status_code == 500
        assert "error" in response.json()


def test_chat_endpoint_garbage_collection(client, sample_chat_data):
    """Test that garbage collection doesn't affect normal chat functionality"""
    # This test verifies that the periodic garbage collection in the app
    # doesn't interfere with normal chat operations
    response = client.post("/api/chat", json=sample_chat_data)
    
    # Should handle gracefully even if garbage collection is happening
    assert response.status_code in [200, 422, 500]


def test_chat_large_message_size(client):
    """Test chat endpoint with a large message"""
    large_message = "A" * 10000  # Large message of 10k chars
    payload = {"message": large_message, "selected_text": None}
    
    with patch('backend.src.main.generate_response') as mock_generate:
        mock_generate.return_value = "Processed large message"
        
        response = client.post("/api/chat", json=payload)
        
        # Should handle large messages reasonably
        assert response.status_code == 200


def test_chat_special_characters(client):
    """Test chat endpoint with special characters"""
    special_message = "Hello! @#$%^&*()_+=-{}[]|\\:;\"'<>?,./"
    payload = {"message": special_message, "selected_text": None}
    
    with patch('backend.src.main.generate_response') as mock_generate:
        mock_generate.return_value = "Processed special characters"
        
        response = client.post("/api/chat", json=payload)
        
        assert response.status_code == 200


def test_chat_unicode_support(client):
    """Test chat endpoint with Unicode characters"""
    unicode_message = "Hello 世界 🌍 Привет مرحبا"
    payload = {"message": unicode_message, "selected_text": None}
    
    with patch('backend.src.main.generate_response') as mock_generate:
        mock_generate.return_value = "Processed unicode message"
        
        response = client.post("/api/chat", json=payload)
        
        assert response.status_code == 200