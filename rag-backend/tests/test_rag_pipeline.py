"""
Basic tests for the RAG Pipeline functions
"""
import pytest
import os
from unittest.mock import patch, MagicMock
from main import (
    get_all_urls,
    extract_text_from_url,
    chunk_text,
    embed,
    create_collection,
    save_chunk_to_qdrant,
    is_duplicate,
    compute_content_hash,
    handle_cohere_rate_limit,
    handle_crawling_errors_with_retry,
    handle_qdrant_errors_with_retry,
    comprehensive_error_logging
)


def test_compute_content_hash():
    """Test the content hash function"""
    content = "This is a test content"
    hash1 = compute_content_hash(content)
    hash2 = compute_content_hash(content)
    assert hash1 == hash2  # Same content should produce same hash
    
    hash3 = compute_content_hash("Different content")
    assert hash1 != hash3  # Different content should produce different hash


def test_chunk_text_basic():
    """Test basic text chunking functionality"""
    text = "This is a test sentence. " * 50  # Create a longer text
    chunks = chunk_text(text, chunk_size_min=20, chunk_size_max=50, overlap_size=10)
    
    assert len(chunks) > 0
    assert all(len(chunk) > 0 for chunk in chunks)


def test_chunk_text_empty():
    """Test chunking with empty text"""
    chunks = chunk_text("")
    assert chunks == []


def test_chunk_text_small():
    """Test chunking with text smaller than minimum chunk size"""
    text = "Small text"
    chunks = chunk_text(text, chunk_size_min=20, chunk_size_max=50, overlap_size=10)
    # Should return the small text as a single chunk
    assert len(chunks) == 1


@patch('main.requests.get')
def test_extract_text_from_url(mock_get):
    """Test extracting text from a URL with mocked response"""
    mock_response = MagicMock()
    mock_response.text = """
    <html>
        <head><title>Test Page</title></head>
        <body>
            <main>
                <p>This is the main content of the test page.</p>
            </main>
        </body>
    </html>
    """
    mock_response.raise_for_status.return_value = None
    mock_get.return_value = mock_response
    
    result = extract_text_from_url("http://example.com")
    
    assert result["title"] == "Test Page"
    assert "main content" in result["text"]


def test_get_all_urls():
    """Test getting all URLs from a base URL"""
    # This is hard to test without a real site, so we'll test error handling
    urls = get_all_urls("https://invalid-url-for-testing.com")
    # Should return empty list or handle gracefully
    assert isinstance(urls, list)


if __name__ == "__main__":
    pytest.main()