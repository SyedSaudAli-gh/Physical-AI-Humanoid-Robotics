# Qdrant vector database setup for RAG functionality
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional, Any
import os
import uuid
from pydantic import BaseModel

# Configuration for Qdrant
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

class VectorDB:
    """
    Vector database interface for RAG functionality using Qdrant
    """
    def __init__(self):
        # Initialize Qdrant client
        if QDRANT_API_KEY:
            self.client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY
            )
        else:
            # For local development
            self.client = QdrantClient(host="localhost", port=6333)
        
        # Define the collection name for textbook content
        self.collection_name = "textbook_content"
    
    def initialize_collection(self, vector_size: int = 1024):
        """
        Initialize the collection for storing textbook content vectors
        """
        try:
            # Check if collection already exists
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except:
            # Create a new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,  # Size of the embedding vectors (adjust based on your embedding model)
                    distance=models.Distance.COSINE  # Similarity metric
                )
            )
            print(f"Created new collection '{self.collection_name}'")
    
    def store_embeddings(self, content_id: str, embedding: List[float], metadata: Dict[str, Any]):
        """
        Store embeddings for content in the vector database
        """
        points = [
            models.PointStruct(
                id=content_id,
                vector=embedding,
                payload={
                    "content_id": content_id,
                    "metadata": metadata
                }
            )
        ]
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
    
    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar content based on embedding
        """
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )
        
        results = []
        for point in search_result:
            results.append({
                "content_id": point.payload["content_id"],
                "metadata": point.payload["metadata"],
                "similarity_score": point.score
            })
        
        return results
    
    def delete_content(self, content_id: str):
        """
        Delete content from the vector database
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(
                points=[content_id]
            )
        )
    
    def update_content(self, content_id: str, new_embedding: List[float], new_metadata: Dict[str, Any]):
        """
        Update existing content in the vector database
        """
        self.client.overwrite_payload(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(
                points=[content_id]
            ),
            payload={
                "content_id": content_id,
                "metadata": new_metadata
            }
        )
        
        # Update the vector as well
        self.client.set_vectors(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=content_id,
                    vector=new_embedding,
                    payload={
                        "content_id": content_id,
                        "metadata": new_metadata
                    }
                )
            ]
        )

class ContentChunker:
    """
    Class to handle document chunking for RAG system
    Implements 512-token chunks with 128-token overlap as specified
    """
    
    def __init__(self, chunk_size: int = 512, overlap: int = 128):
        self.chunk_size = chunk_size
        self.overlap = overlap
    
    def chunk_text(self, text: str) -> List[Dict[str, any]]:
        """
        Split text into overlapping chunks
        Note: This is a simplified implementation using word-based counting
        In a real implementation, you'd use tokenization specific to your model
        """
        words = text.split()
        chunks = []
        
        # Calculate how many words to move forward each iteration
        step_size = self.chunk_size - self.overlap
        
        start_idx = 0
        chunk_id = 0
        
        while start_idx < len(words):
            end_idx = min(start_idx + self.chunk_size, len(words))
            chunk_text = ' '.join(words[start_idx:end_idx])
            
            # Create chunk with metadata
            chunk = {
                'chunk_id': f'chunk_{chunk_id}',
                'text': chunk_text,
                'start_idx': start_idx,
                'end_idx': end_idx,
                'metadata': {
                    'word_count': len(chunk_text.split()),
                    'position': chunk_id
                }
            }
            
            chunks.append(chunk)
            
            # Move to the next chunk position
            start_idx += step_size
            chunk_id += 1
            
            # Ensure we don't get stuck in an infinite loop
            if step_size <= 0 and start_idx < len(words):
                break
        
        return chunks

# Initialize the vector database when module is loaded
vector_db = VectorDB()
chunker = ContentChunker()

if __name__ == "__main__":
    print("Vector database initialized for RAG functionality")
    print(f"Connected to: {QDRANT_URL}")
    print("Collection name: textbook_content")
    print("Ready to store and retrieve content embeddings")