"""
Tool definitions for the RAG Agent that integrate with the retrieval service.
"""
import logging
from typing import List, Dict, Any
from retrieval_service import retrieve
from config import OPENAI_API_KEY
import openai

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Set OpenAI API key
openai.api_key = OPENAI_API_KEY


def retrieve_content_tool(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Tool that uses the retrieval service to find relevant content based on a query.

    Args:
        query (str): The query to search for in the knowledge base
        top_k (int): Number of results to return (default: 5)

    Returns:
        List[Dict[str, Any]]: List of retrieved chunks with their metadata
    """
    try:
        # Use the retrieval service from Spec 2
        from retrieval_service import retrieve

        # Perform the retrieval
        search_result = retrieve(query_text=query, top_k=top_k)

        # Format the results to match the expected structure
        formatted_results = []
        for chunk in search_result.retrieved_chunks:
            formatted_results.append({
                "content": chunk.content,
                "source_url": chunk.source_url,
                "page_title": chunk.page_title,
                "similarity_score": chunk.similarity_score,
                "metadata": chunk.chunk_metadata
            })

        logger.info(f"Retrieved {len(formatted_results)} chunks for query: {query[:50]}...")
        return formatted_results

    except Exception as e:
        logger.error(f"Error in retrieve_content_tool: {e}")
        return []


def get_relevant_context(query: str, selected_text: str = None) -> str:
    """
    Function that gets relevant context for a query, optionally using selected text.

    Args:
        query (str): The main query from the user
        selected_text (str, optional): Text that the user has highlighted/selected

    Returns:
        str: Formatted context string containing relevant content
    """
    try:
        # If selected text is provided, we can incorporate it into the query
        search_query = query
        if selected_text:
            search_query = f"{query} related to: {selected_text}"

        # Retrieve relevant content
        retrieved_chunks = retrieve_content_tool(search_query)

        # Format the context
        context_parts = []
        for chunk in retrieved_chunks:
            context_parts.append(
                f"Source: {chunk['source_url']}\n"
                f"Title: {chunk['page_title']}\n"
                f"Content: {chunk['content']}\n"
                f"Relevance Score: {chunk['similarity_score']}\n"
                "---"
            )

        context = "\n".join(context_parts)
        logger.info(f"Formatted context from {len(retrieved_chunks)} chunks")
        return context

    except Exception as e:
        logger.error(f"Error in get_relevant_context: {e}")
        return ""