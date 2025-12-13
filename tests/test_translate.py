import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import json


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


def test_translate_endpoint_missing_text(client):
    """Test the translate endpoint with missing text parameter"""
    response = client.post("/api/translate", json={"target_lang": "es"})
    assert response.status_code == 422  # Validation error


def test_translate_endpoint_empty_text(client):
    """Test the translate endpoint with empty text"""
    payload = {"text": "", "target_lang": "es"}
    response = client.post("/api/translate", json=payload)
    assert response.status_code == 422


def test_translate_endpoint_empty_target_lang(client):
    """Test the translate endpoint with empty target language"""
    payload = {"text": "Hello", "target_lang": ""}
    response = client.post("/api/translate", json=payload)
    assert response.status_code == 422


def test_translate_different_languages(client):
    """Test translation to different languages"""
    translations = [
        {"text": "Good morning", "target_lang": "fr", "expected": "Bonjour"},
        {"text": "Good evening", "target_lang": "de", "expected": "Guten Abend"},
        {"text": "Thank you", "target_lang": "ja", "expected": "ありがとう"}
    ]
    
    for trans_data in translations:
        with patch('backend.src.main.translate_text') as mock_translate:
            mock_translate.return_value = trans_data["expected"]
            
            response = client.post("/api/translate", json={
                "text": trans_data["text"],
                "target_lang": trans_data["target_lang"]
            })
            
            assert response.status_code == 200
            assert response.json()["translated_text"] == trans_data["expected"]


def test_translate_long_text(client):
    """Test translation of longer text"""
    long_text = "This is a longer text that needs to be translated. " * 10  # Repeat 10 times
    payload = {
        "text": long_text,
        "target_lang": "es"
    }
    
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.return_value = "Texto largo traducido"
        
        response = client.post("/api/translate", json=payload)
        
        assert response.status_code == 200
        assert response.json()["translated_text"] == "Texto largo traducido"


def test_translate_special_characters(client):
    """Test translation with special characters"""
    special_text = "Hello! @#$%^&*()_+=-{}[]|\\:;\"'<>?,./"
    payload = {
        "text": special_text,
        "target_lang": "fr"
    }
    
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.return_value = "Bonjour ! @#$%^&*()_+=-{}[]|\\:;\"'<>?,./"
        
        response = client.post("/api/translate", json=payload)
        
        assert response.status_code == 200
        assert "translated_text" in response.json()


def test_translate_unicode_text(client):
    """Test translation with Unicode characters"""
    unicode_text = "Hello 世界 🌍 Привет مرحبا"
    payload = {
        "text": unicode_text,
        "target_lang": "es"
    }
    
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.return_value = "Hola 世界 🌍 Привет مرحبا"
        
        response = client.post("/api/translate", json=payload)
        
        assert response.status_code == 200
        assert "translated_text" in response.json()


def test_translate_endpoint_error_handling(client, sample_translation_data):
    """Test the translate endpoint error handling"""
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.side_effect = Exception("Translation API Error")
        
        response = client.post("/api/translate", json=sample_translation_data)
        
        assert response.status_code == 500
        assert "error" in response.json()


def test_translate_unsupported_language(client, sample_translation_data):
    """Test translation with unsupported language code"""
    sample_translation_data["target_lang"] = "xx"  # Invalid language code
    
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.side_effect = ValueError("Unsupported language: xx")
        
        response = client.post("/api/translate", json=sample_translation_data)
        
        assert response.status_code == 500
        assert "error" in response.json()


def test_translate_source_detection(client):
    """Test translation with automatic source language detection"""
    payload = {
        "text": "Bonjour le monde",
        "target_lang": "en"
    }
    
    with patch('backend.src.main.translate_text') as mock_translate:
        mock_translate.return_value = "Hello world"
        
        response = client.post("/api/translate", json=payload)
        
        assert response.status_code == 200
        assert response.json()["translated_text"] == "Hello world"