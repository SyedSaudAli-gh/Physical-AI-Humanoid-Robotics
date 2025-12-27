"""
Agent functionality tests for the RAG Agent application.
"""
import pytest
from unittest.mock import patch, MagicMock
from models import ChatRequest, ChatResponse
from agent import RAGAgent, process_chat_request


def test_rag_agent_initialization():
    """Test that the RAG agent initializes correctly."""
    agent = RAGAgent()
    assert agent.model == "gpt-4-turbo-preview"


@patch('agent.client')
def test_answer_question_success(mock_openai_client):
    """Test that the agent successfully answers a question."""
    # Mock the OpenAI response
    mock_response = MagicMock()
    mock_response.choices = [MagicMock()]
    mock_response.choices[0].message.content = "This is a test answer."
    
    mock_openai_client.chat.completions.create.return_value = mock_response
    
    agent = RAGAgent()
    chat_request = ChatRequest(query="Test query?")
    
    response = agent.answer_question(chat_request)
    
    assert isinstance(response, ChatResponse)
    assert "test answer" in response.answer.lower()


def test_process_chat_request():
    """Test the process_chat_request function."""
    chat_request = ChatRequest(query="What is ROS?")
    
    # Since the actual function relies on external services, 
    # we'll just verify it takes the right input and returns the right output type
    # For a complete test, we'd need to mock the dependencies
    with patch('agent.rag_agent.answer_question') as mock_method:
        mock_method.return_value = ChatResponse(answer="Test answer", sources=[])
        result = process_chat_request(chat_request)
        
        assert isinstance(result, ChatResponse)
        assert result.answer == "Test answer"


def test_chat_request_validation():
    """Test ChatRequest model validation."""
    # Valid request
    valid_request = ChatRequest(query="What is AI?")
    assert valid_request.query == "What is AI?"
    
    # Valid request with selected text
    valid_request_with_selection = ChatRequest(
        query="What is AI?",
        selected_text="Artificial Intelligence"
    )
    assert valid_request_with_selection.query == "What is AI?"
    assert valid_request_with_selection.selected_text == "Artificial Intelligence"


def test_chat_response_validation():
    """Test ChatResponse model validation."""
    from models import SourceReference
    
    # Valid response
    source_ref = SourceReference(
        source_url="http://example.com",
        page_title="Test Page",
        snippet="Test snippet",
        relevance_score=0.9
    )
    
    response = ChatResponse(
        answer="This is the answer",
        sources=[source_ref]
    )
    
    assert response.answer == "This is the answer"
    assert len(response.sources) == 1
    assert response.sources[0].page_title == "Test Page"