"""
FastAPI application for the RAG Agent with POST /api/chat endpoint and CORS support.
"""
import logging
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import ValidationError
import asyncio

from models import ChatRequest, ChatResponse
from agent import process_chat_request
from config import API_HOST, API_PORT, API_RELOAD

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="An AI agent using OpenAI Agents SDK with FastAPI backend that answers user questions about the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for frontend access if needed
    # expose_headers=["Access-Control-Allow-Origin"]
)

@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest) -> ChatResponse:
    """
    POST endpoint to accept user questions and return AI-generated answers
    based on the textbook content with source references.
    
    Args:
        chat_request (ChatRequest): The incoming request with query and optional selected_text
        
    Returns:
        ChatResponse: The response with answer and source references
    """
    try:
        logger.info(f"Received chat request for query: {chat_request.query[:50]}...")
        
        # Process the request using the RAG agent
        response = process_chat_request(chat_request)
        
        logger.info(f"Successfully processed chat request, returning answer with {len(response.sources)} sources")
        return response
        
    except ValidationError as ve:
        logger.error(f"Validation error in chat endpoint: {ve}")
        raise HTTPException(status_code=422, detail=str(ve))
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/")
async def root():
    """
    Health check endpoint.
    """
    return {"message": "RAG Agent API is running"}

@app.get("/health")
async def health_check():
    """
    Detailed health check endpoint.
    """
    return {
        "status": "healthy",
        "service": "RAG Agent API",
        "version": "1.0.0"
    }

# For running with uvicorn
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=API_HOST, port=API_PORT, reload=API_RELOAD)