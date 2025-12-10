from sqlalchemy.orm import Session
from typing import List, Dict, Any
from ..models.rag_index import RagIndex
from ..models.interaction_log import InteractionLog
from ..rag.vector_db import vector_db, chunker
from ..rag.cohere_embedder import embedding_service
from .cache_service import CacheService
from datetime import datetime
import json

class RAGService:
    """
    Service for handling Retrieval Augmented Generation (RAG) functionality
    """
    
    def __init__(self, db: Session):
        self.db = db
    
    def index_content(self, content_id: str, content_text: str, metadata: Dict[str, Any]) -> bool:
        """
        Index content for RAG by creating embeddings and storing in vector database
        """
        try:
            # Chunk the content using the 512-token chunker with 128-token overlap
            chunks = chunker.chunk_text(content_text)
            
            # Process each chunk
            for chunk in chunks:
                # Generate embedding for the chunk text
                embedding = embedding_service.get_embedding_for_content(
                    chunk['text'], 
                    language=metadata.get('language', 'en'),
                    content_type='search_document'
                )
                
                # Store in vector database
                vector_db.store_embeddings(
                    content_id=f"{content_id}_chunk_{chunk['chunk_id']}",
                    embedding=embedding,
                    metadata={
                        **metadata,
                        'chunk_id': chunk['chunk_id'],
                        'chunk_text': chunk['text'],
                        'position': chunk['metadata']['position']
                    }
                )
                
                # Also store in the database for record keeping
                rag_index = RagIndex(
                    id=f"{content_id}_chunk_{chunk['chunk_id']}",
                    content_id=content_id,
                    embedding=embedding,
                    chunk_text=chunk['text'],
                    chunk_metadata=json.dumps({
                        **metadata,
                        'chunk_id': chunk['chunk_id'],
                        'position': chunk['metadata']['position']
                    })
                )
                self.db.add(rag_index)
            
            self.db.commit()
            return True
        except Exception as e:
            print(f"Error indexing content: {str(e)}")
            self.db.rollback()
            return False
    
    def query_content(self, query: str, selected_text: str = None,
                     chapter_id: str = None, include_context: bool = True,
                     limit: int = 5) -> Dict[str, Any]:
        """
        Query the RAG system with book context
        """
        # Check cache first
        cached_response = CacheService.get_cached_response(query, selected_text, chapter_id)
        if cached_response:
            # Add cache hit indicator
            cached_response['cache_hit'] = True
            return cached_response

        try:
            # Generate embedding for the query
            query_embedding = embedding_service.get_embedding_for_content(
                query,
                language='en',
                content_type='search_query'
            )

            # Search for similar content in the vector database
            search_results = vector_db.search_similar(query_embedding, limit=limit)

            # Prepare response
            response = {
                'query': query,
                'response': '',
                'sources': [],
                'timestamp': datetime.now().isoformat()
            }

            if search_results:
                # Generate response based on retrieved context
                context_texts = []
                for result in search_results:
                    context_texts.append(result['metadata']['chunk_text'])
                    response['sources'].append({
                        'chapter_id': result['metadata'].get('chapter_id'),
                        'module_id': result['metadata'].get('module_id'),
                        'relevance_score': result['similarity_score']
                    })

                # Generate response using the retrieved context
                # In a real implementation, this would call an LLM with the context
                response['response'] = self._generate_response_with_context(
                    query,
                    context_texts,
                    selected_text
                )
            else:
                response['response'] = "I couldn't find relevant information to answer your question."

            # Cache the response for future queries
            CacheService.cache_response(query, response, selected_text, chapter_id)

            return response
        except Exception as e:
            print(f"Error querying content: {str(e)}")
            error_response = {
                'query': query,
                'response': 'Error processing your query. Please try again later.',
                'sources': [],
                'timestamp': datetime.now().isoformat()
            }

            # Still cache error responses to prevent repeated error processing
            CacheService.cache_response(query, error_response, selected_text, chapter_id)

            return error_response
    
    def _generate_response_with_context(self, query: str, context_texts: List[str], 
                                      selected_text: str = None) -> str:
        """
        Generate response using the retrieved context
        In a real implementation, this would call an LLM with the context
        """
        # This is a simplified implementation
        # In a real system, you would send the query + context to an LLM
        
        # Combine context texts
        context = " ".join(context_texts[:3])  # Use top 3 results
        
        # Create a response based on context
        if selected_text:
            response = f"Based on the selected text and related content: {context[:500]}... "
            response += f"The information relevant to your query '{query}' is likely found in this context."
        else:
            response = f"Based on the textbook content: {context[:500]}... "
            response += f"This information relates to your query about '{query}'."
        
        return response
    
    def log_interaction(self, user_id: str, query: str, response: str, 
                       selected_text: str = None, chapter_id: str = None) -> bool:
        """
        Log interaction for analytics and improvement
        """
        try:
            interaction = InteractionLog(
                user_id=user_id,
                interaction_type="chat_query",
                content_id=chapter_id,
                query_text=query,
                response_text=response,
                selected_text=selected_text,
                personalization_applied=None,  # Will add if personalization is applied
                session_id="session_placeholder"  # Should be passed from the session
            )
            self.db.add(interaction)
            self.db.commit()
            return True
        except Exception as e:
            print(f"Error logging interaction: {str(e)}")
            self.db.rollback()
            return False

# Example usage
if __name__ == "__main__":
    print("RAG service created")