"""
Unit tests for the retrieval functionality
"""
import pytest
from unittest.mock import patch, MagicMock
from retrieval_service import (
    validate_query,
    convert_query_to_embedding,
    search_qdrant,
    retrieve,
    QueryParameters
)


def test_validate_query_valid():
    """Test that valid queries are validated correctly"""
    assert validate_query("This is a valid query") == True


def test_validate_query_empty():
    """Test that empty queries are rejected"""
    assert validate_query("") == False


def test_validate_query_whitespace():
    """Test that queries with only whitespace are rejected"""
    assert validate_query("   ") == False


def test_validate_query_none():
    """Test that None queries are rejected"""
    assert validate_query(None) == False


@patch('retrieval_service.co')
def test_convert_query_to_embedding(mock_cohere):
    """Test converting a query to embedding"""
    mock_cohere.embed.return_value = MagicMock()
    mock_cohere.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]
    
    result = convert_query_to_embedding("Test query")
    assert result == [0.1, 0.2, 0.3]
    mock_cohere.embed.assert_called_once()


@patch('retrieval_service.qdrant_client')
def test_search_qdrant(mock_qdrant_client):
    """Test searching in Qdrant"""
    # Mock the search results
    mock_result = MagicMock()
    mock_result.id = "test_id"
    mock_result.score = 0.9
    mock_result.payload = {
        "content": "test content",
        "source_url": "http://example.com",
        "page_title": "Test Page"
    }
    mock_qdrant_client.search.return_value = [mock_result]
    
    results = search_qdrant([0.1, 0.2, 0.3], top_k=1)
    assert len(results) == 1
    assert results[0].content == "test content"
    assert results[0].similarity_score == 0.9


@patch('retrieval_service.validate_query')
@patch('retrieval_service.convert_query_to_embedding')
@patch('retrieval_service.search_qdrant')
def test_retrieve_function(mock_search, mock_embed, mock_validate):
    """Test the retrieve function"""
    mock_validate.return_value = True
    mock_embed.return_value = [0.1, 0.2, 0.3]
    
    # Mock search results
    from retrieval_service import RetrievedChunk
    mock_chunk = RetrievedChunk(
        chunk_id="test_id",
        content="test content",
        similarity_score=0.9,
        source_url="http://example.com",
        page_title="Test Page"
    )
    mock_search.return_value = [mock_chunk]
    
    result = retrieve("Test query")
    
    assert result.query_text == "Test query"
    assert len(result.retrieved_chunks) == 1
    assert result.retrieved_chunks[0].content == "test content"


def test_retrieve_with_invalid_query():
    """Test that retrieve raises error for invalid query"""
    with pytest.raises(ValueError):
        retrieve("")


if __name__ == "__main__":
    pytest.main()