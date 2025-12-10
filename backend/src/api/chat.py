from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List, Dict, Any
from database import get_db
from services.rag_service import RAGService


router = APIRouter(prefix="/chat", tags=["chat"])

class QueryRequest(BaseModel):
    query: str
    selected_text: str = None  # Text selected by the user in the textbook

class QueryResponse(BaseModel):
    query: str
    answer: str
    sources: List[Dict[str, Any]]
    selected_text_context: str = None

class ProcessChapterRequest(BaseModel):
    chapter_id: int

class ProcessChapterResponse(BaseModel):
    chapter_id: int
    chapter_title: str
    chunks_processed: int
    status: str

@router.post("/query", response_model=QueryResponse)
def query_knowledge_base(request: QueryRequest, db: Session = Depends(get_db)):
    """
    Query the RAG system for answers based on textbook content
    """
    try:
        service = RAGService(db)
        result = service.query_knowledge_base(request.query, request.selected_text)
        
        return QueryResponse(
            query=result["query"],
            answer=result["answer"],
            sources=result["sources"],
            selected_text_context=result["selected_text_context"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"RAG query failed: {str(e)}")

@router.post("/process-chapter", response_model=ProcessChapterResponse)
def process_chapter_for_rag(request: ProcessChapterRequest, db: Session = Depends(get_db)):
    """
    Process a chapter and store its content in the vector database for RAG
    """
    try:
        service = RAGService(db)
        result = service.process_and_store_chapter(request.chapter_id)
        
        return ProcessChapterResponse(**result)
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chapter processing failed: {str(e)}")