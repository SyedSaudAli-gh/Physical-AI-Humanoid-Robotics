"""
Pydantic models for request/response validation in the RAG Agent application.
"""
from pydantic import BaseModel, Field
from typing import List, Optional


class ChatRequest(BaseModel):
    """
    Represents a user's request to the chat endpoint with query and optional selected_text
    """
    query: str = Field(..., description="The main question from the user", min_length=1)
    selected_text: Optional[str] = Field(None, description="Optional text that the user has highlighted or selected")


class SourceReference(BaseModel):
    """
    Represents a reference to the original content that contributed to the agent's answer
    """
    source_url: str = Field(..., description="URL of the original source")
    page_title: str = Field(..., description="Title of the original page")
    snippet: str = Field(..., description="Relevant snippet from the source")
    relevance_score: float = Field(..., description="Relevance score of this source to the query", ge=0.0, le=1.0)


class ChatResponse(BaseModel):
    """
    Represents the agent's response with answer and array of source references
    """
    answer: str = Field(..., description="The agent's response to the user's query", min_length=1)
    sources: List[SourceReference] = Field(default_factory=list, description="List of sources used to generate the answer")