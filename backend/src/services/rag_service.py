import openai
from typing import List, Dict, Any
from sqlalchemy.orm import Session
from rag.vector_db import vector_db, chunker
from rag.cohere_embedder import embedding_service
from models.chapter import Chapter
from config import settings


class RAGService:
    def __init__(self, db: Session):
        self.db = db
        openai.api_key = settings.openai_api_key

    def process_and_store_chapter(self, chapter_id: int):
        """Process a chapter and store its chunks in the vector database"""
        # Get the chapter content
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")
            
        # Chunk the content
        chunks = chunker.chunk_text(chapter.content)
        
        # Process each chunk
        for chunk in chunks:
            # Generate embedding for the chunk
            embedding = embedding_service.get_embedding_for_content(
                chunk['text'], 
                language="en", 
                content_type="document"
            )
            
            # Prepare metadata
            metadata = {
                'chapter_id': chapter.id,
                'chapter_title': chapter.title,
                'module_id': chapter.module_id,
                'chunk_position': chunk['metadata']['position'],
                'word_count': chunk['metadata']['word_count']
            }
            
            # Store in vector database
            vector_db.store_embeddings(
                content_id=chunk['chunk_id'],
                embedding=embedding,
                metadata=metadata
            )
        
        return {
            "chapter_id": chapter.id,
            "chapter_title": chapter.title,
            "chunks_processed": len(chunks),
            "status": "success"
        }

    def query_knowledge_base(self, query: str, selected_text: str = None) -> Dict[str, Any]:
        """Query the knowledge base using RAG approach"""
        
        # Combine query with selected text if available
        full_query = query
        if selected_text:
            full_query = f"Context: {selected_text}\nQuestion: {query}"
        
        # Generate embedding for the query
        query_embedding = embedding_service.get_embedding_for_content(
            full_query,
            language="en",
            content_type="query"
        )
        
        # Search for similar content in the vector database
        search_results = vector_db.search_similar(query_embedding, limit=5)
        
        if not search_results:
            return {
                "query": query,
                "answer": "I couldn't find relevant information in the textbook to answer your question.",
                "sources": [],
                "selected_text_context": selected_text
            }
        
        # Build context from search results
        context_parts = []
        sources = []
        
        for result in search_results:
            metadata = result['metadata']
            context_parts.append(metadata.get('text', ''))  # Use text from metadata if available
            
            sources.append({
                'chapter_id': metadata.get('chapter_id'),
                'chapter_title': metadata.get('chapter_title'),
                'similarity_score': result['similarity_score']
            })
        
        context = "\n\n".join(context_parts)
        
        # Generate an answer using OpenAI
        prompt = f"""
        You are an expert on Physical AI & Humanoid Robotics. Use the following context to answer the question.
        If the context doesn't contain enough information to answer the question, say so.
        
        Context:
        {context}
        
        Question: {query}
        
        Answer (provide specific information from the context if available, otherwise say you don't have enough information):
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are an expert assistant for a Physical AI & Humanoid Robotics textbook. Provide accurate, helpful answers based on the textbook content. Always cite specific information from the provided context."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )
            
            answer = response.choices[0].message.content.strip()
            
            return {
                "query": query,
                "answer": answer,
                "sources": sources,
                "selected_text_context": selected_text
            }
        except Exception as e:
            return {
                "query": query,
                "answer": f"Error generating answer: {str(e)}",
                "sources": sources,
                "selected_text_context": selected_text
            }

    def get_relevant_content(self, query: str, limit: int = 3) -> List[Dict[str, Any]]:
        """Get relevant content chunks based on a query"""
        
        query_embedding = embedding_service.get_embedding_for_content(
            query,
            language="en",
            content_type="query"
        )
        
        search_results = vector_db.search_similar(query_embedding, limit=limit)
        
        content_list = []
        for result in search_results:
            content_list.append({
                'chunk_id': result['content_id'],
                'metadata': result['metadata'],
                'similarity_score': result['similarity_score']
            })
        
        return content_list