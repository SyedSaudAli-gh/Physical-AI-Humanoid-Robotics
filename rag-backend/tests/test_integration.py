"""
Integration tests with retrieval service for the RAG Agent application.
"""
import pytest
from unittest.mock import patch, MagicMock
from models import ChatRequest
from agent import process_chat_request


@patch('agent.get_relevant_context')
@patch('agent.client')
def test_integration_with_mocked_services(mock_openai_client, mock_get_context):
    """Test the full integration flow with mocked external services."""
    # Mock the context retrieval
    mock_get_context.return_value = "This is relevant context for the query."
    
    # Mock the OpenAI response
    mock_response = MagicMock()
    mock_response.choices = [MagicMock()]
    mock_response.choices[0].message.content = "Based on the context, here is the answer."
    
    mock_openai_client.chat.completions.create.return_value = mock_response
    
    # Create a test request
    chat_request = ChatRequest(
        query="What is the main concept discussed in the textbook?",
        selected_text="main concept"
    )
    
    # Process the request
    response = process_chat_request(chat_request)
    
    # Verify the response
    assert "answer" in response.dict()
    assert "based on the context" in response.answer.lower()
    assert hasattr(response, 'sources')
    
    # Verify that the context retrieval was called with the right parameters
    mock_get_context.assert_called_once_with(
        chat_request.query,
        chat_request.selected_text
    )


@patch('tools.retrieve_content_tool')
def test_tool_integration(mock_retrieve_tool):
    """Test the integration with the retrieval tool."""
    from tools import get_relevant_context
    
    # Mock the retrieval tool response
    mock_chunks = [
        {
            "content": "This is a relevant content chunk.",
            "source_url": "http://example.com/page1",
            "page_title": "Example Page",
            "similarity_score": 0.95,
            "metadata": {}
        }
    ]
    mock_retrieve_tool.return_value = mock_chunks
    
    # Test the function
    context = get_relevant_context("test query", "selected text")
    
    # Verify the context was constructed properly
    assert "This is a relevant content chunk." in context
    assert "http://example.com/page1" in context
    assert "Example Page" in context
    
    # Verify the tool was called with the right query
    mock_retrieve_tool.assert_called_once_with("test query related to: selected text")


@patch('agent.client')
def test_error_handling_in_agent(mock_openai_client):
    """Test error handling in the agent when OpenAI API fails."""
    from agent import RAGAgent
    from models import ChatRequest
    
    # Mock an exception from the OpenAI API
    mock_openai_client.chat.completions.create.side_effect = Exception("API Error")
    
    agent = RAGAgent()
    chat_request = ChatRequest(query="Test query?")
    
    # Expect an exception to be raised
    with pytest.raises(Exception) as exc_info:
        agent.answer_question(chat_request)
    
    assert "API Error" in str(exc_info.value)


def test_models_validation():
    """Test that the models properly validate input."""
    from models import ChatRequest, SourceReference
    
    # Test that ChatRequest requires a query
    with pytest.raises(ValueError):
        ChatRequest(query="")
    
    # Test that SourceReference validates relevance_score
    with pytest.raises(ValueError):
        SourceReference(
            source_url="http://example.com",
            page_title="Test",
            snippet="Test",
            relevance_score=1.5  # Above 1.0
        )
    
    with pytest.raises(ValueError):
        SourceReference(
            source_url="http://example.com",
            page_title="Test",
            snippet="Test",
            relevance_score=-0.5  # Below 0.0
        )
    
    # Valid SourceReference should work
    valid_source = SourceReference(
        source_url="http://example.com",
        page_title="Test",
        snippet="Test",
        relevance_score=0.8
    )
    assert valid_source.relevance_score == 0.8