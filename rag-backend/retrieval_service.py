"""
RAG Pipeline Retrieval Service

This module implements the retrieval functionality for the RAG pipeline,
converting text queries to embeddings and searching in Qdrant for relevant chunks.
"""

import os
import time
from typing import List, Dict, Optional, Any
from dataclasses import dataclass
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging
import google.generativeai as genai
from mock_embedding import mock_embed_text

# Load environment variables
load_dotenv()

# Import configuration
from config import DEFAULT_TOP_K, RESPONSE_TIMEOUT_MS, MIN_SCORE_THRESHOLD, COLLECTION_NAME, GEMINI_API_KEY

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configure Google Generative AI
genai.configure(api_key=GEMINI_API_KEY)

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=10  # 10 seconds timeout
)


@dataclass
class Query:
    """Represents a user's text input that needs to be matched against stored content"""
    query_text: str
    query_embedding: Optional[List[float]] = None
    timestamp: Optional[float] = None
    top_k: int = DEFAULT_TOP_K


@dataclass
class RetrievedChunk:
    """A text chunk from the knowledge base that matches the query, with similarity score and metadata"""
    chunk_id: str
    content: str
    similarity_score: float
    source_url: str
    page_title: str
    chunk_metadata: Optional[Dict[str, Any]] = None
    rank: int = 0


@dataclass
class SearchResult:
    """A collection of retrieved chunks with their scores and metadata, returned in response to a query"""
    query_id: str
    retrieved_chunks: List[RetrievedChunk]
    total_results: int
    search_time_ms: float
    query_text: str


@dataclass
class QueryParameters:
    """Configuration parameters for the retrieval process"""
    top_k: int = DEFAULT_TOP_K
    response_timeout_ms: int = RESPONSE_TIMEOUT_MS
    min_score_threshold: float = MIN_SCORE_THRESHOLD
    collection_name: str = COLLECTION_NAME


def validate_query(query_text: str) -> bool:
    """
    Validate input queries
    """
    if not query_text or not query_text.strip():
        return False
    if len(query_text.strip()) == 0:
        return False
    return True


def convert_query_to_embedding(query_text: str) -> List[float]:
    """
    Convert text query to embeddings using Google's embedding service or mock if quota exceeded
    """
    try:
        # Try to use the actual Google embedding service
        result = genai.embed_content(
            model="models/embedding-001",  # Google's embedding model
            content=[query_text],
            task_type="retrieval_query"  # Specify this is for retrieval
        )
        return result['embedding'][0]  # Return the first (and only) embedding
    except Exception as e:
        logger.warning(f"Error using Google embedding service: {e}. Using mock embedding.")
        # If there's an error (like quota exceeded), use mock embedding
        return mock_embed_text(query_text)


def handle_gemini_unavailability():
    """
    Handle Gemini API unavailability
    """
    logger.error("Gemini API is unavailable. Please check your API key and connection.")
    raise ConnectionError("Gemini API is currently unavailable")


def handle_qdrant_unavailability():
    """
    Handle Qdrant service unavailability
    """
    logger.error("Qdrant service is unavailable. Please check your connection and collection configuration.")
    raise ConnectionError("Qdrant service is currently unavailable")


def ensure_collection_exists(collection_name: str = COLLECTION_NAME):
    """
    Ensure the Qdrant collection exists, create it if it doesn't
    """
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]
        
        if collection_name not in collection_names:
            logger.info(f"Collection '{collection_name}' does not exist, creating it...")
            
            # Create the collection with appropriate configuration
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Google embeddings size for 'models/embedding-001'
                    distance=models.Distance.COSINE
                )
            )
            
            logger.info(f"Collection '{collection_name}' created successfully")
        else:
            logger.info(f"Collection '{collection_name}' already exists")
            
    except Exception as e:
        logger.error(f"Error ensuring collection exists: {e}")
        raise e


def search_qdrant(query_embedding: List[float], top_k: int = DEFAULT_TOP_K,
                  collection_name: str = COLLECTION_NAME) -> List[RetrievedChunk]:
    """
    Perform similarity search in Qdrant for top-k chunks
    """
    try:
        # Ensure collection exists before searching
        ensure_collection_exists(collection_name)
        
        # Perform the search in Qdrant
        search_response = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
            score_threshold=MIN_SCORE_THRESHOLD  # Apply threshold at search level
        )

        # Convert search results to RetrievedChunk objects
        # The response has a 'points' attribute containing the results
        retrieved_chunks = []
        for i, result in enumerate(search_response.points):
            payload = result.payload
            chunk = RetrievedChunk(
                chunk_id=result.id,
                content=payload.get("content", ""),
                similarity_score=result.score,
                source_url=payload.get("source_url", ""),
                page_title=payload.get("page_title", ""),
                chunk_metadata=payload,
                rank=i+1
            )
            retrieved_chunks.append(chunk)

        return retrieved_chunks
    except Exception as e:
        logger.error(f"Error searching Qdrant: {e}")
        handle_qdrant_unavailability()
        raise e


def handle_edge_cases(query_text: str, search_results: List[RetrievedChunk]) -> List[RetrievedChunk]:
    """
    Handle edge cases in the search results
    """
    # This function is called internally by retrieve function to handle edge cases
    # For now, it just returns the results as is, but can be extended for more complex handling
    return search_results


def retrieve(query_text: str, top_k: int = DEFAULT_TOP_K,
             min_score_threshold: float = MIN_SCORE_THRESHOLD) -> SearchResult:
    """
    Orchestrates the query processing flow (validate → embed → search → return results)
    """
    start_time = time.time()

    # Validate the query
    if not validate_query(query_text):
        raise ValueError("Invalid query: query is empty or contains only whitespace")

    # Convert query to embedding
    query_embedding = convert_query_to_embedding(query_text)

    # Search in Qdrant
    retrieved_chunks = search_qdrant(query_embedding, top_k)

    # Handle edge case: no results found
    if not retrieved_chunks:
        logger.info(f"No results found for query: {query_text}")

    # Apply edge case handling
    retrieved_chunks = handle_edge_cases(query_text, retrieved_chunks)

    # Calculate search time
    search_time_ms = (time.time() - start_time) * 1000

    # Create and return search result
    result = SearchResult(
        query_id=f"query_{int(start_time)}",  # Simple ID based on timestamp
        retrieved_chunks=retrieved_chunks,
        total_results=len(retrieved_chunks),
        search_time_ms=search_time_ms,
        query_text=query_text
    )

    return result