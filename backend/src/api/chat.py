from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from typing import Optional, Dict, Any
from pydantic import BaseModel
from ..database import get_db
from ..services.rag_service import RAGService

router = APIRouter(prefix="/chat", tags=["chat"])

# Request model for chat queries
class ChatQueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    chapter_id: Optional[str] = None
    include_context: Optional[bool] = True

@router.post("/query", response_model=Dict[str, Any])
def query_chat(request: ChatQueryRequest, db: Session = Depends(get_db)):
    """
    Query the RAG system with book context
    """
    try:
        rag_service = RAGService(db)
        
        # Perform the query
        result = rag_service.query_content(
            query=request.query,
            selected_text=request.selected_text,
            chapter_id=request.chapter_id,
            include_context=request.include_context
        )
        
        # Log the interaction for analytics
        # Note: In a real implementation, we would extract the user ID from the request
        # For now, using a placeholder
        user_id = "placeholder_user_id"  # Should come from authentication
        rag_service.log_interaction(
            user_id=user_id,
            query=request.query,
            response=result.get('response', 'No response generated'),
            selected_text=request.selected_text,
            chapter_id=request.chapter_id
        )
        
        return result
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing chat query: {str(e)}"
        )