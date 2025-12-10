# Cohere configuration for embeddings with multilingual support
import cohere
from typing import List, Dict, Any
import os
from ..config import settings

class CohereEmbedder:
    """
    Cohere embedder for generating text embeddings with multilingual support
    """
    
    def __init__(self):
        # Initialize Cohere client
        api_key = settings.cohere_api_key
        if not api_key:
            raise ValueError("COHERE_API_KEY is required for embeddings")
        
        self.client = cohere.Client(api_key)
        self.model = "embed-multilingual-v3.0"  # Cohere's multilingual embedding model
    
    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        input_type can be: "search_document", "search_query", "classification", "clustering"
        """
        response = self.client.embed(
            texts=texts,
            model=self.model,
            input_type=input_type
        )
        
        return [embedding for embedding in response.embeddings]
    
    def embed_text(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text
        """
        embeddings = self.generate_embeddings([text], input_type)
        return embeddings[0]  # Return the first (and only) embedding
    
    def embed_multiple_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for multiple texts with batching if needed
        Cohere API has limits, so we might need to process in batches
        """
        # Note: In a real implementation, you'd check Cohere's API limits
        # and potentially process in batches
        return self.generate_embeddings(texts, input_type)

class MultilingualEmbeddingService:
    """
    Service for handling multilingual embeddings with support for Urdu and other languages
    """
    
    def __init__(self):
        self.embedder = CohereEmbedder()
    
    def get_embedding_for_content(self, content: str, language: str = "en", content_type: str = "document") -> List[float]:
        """
        Get embedding for content with specified language
        """
        input_type = self._get_input_type(content_type)
        return self.embedder.embed_text(content, input_type)
    
    def _get_input_type(self, content_type: str) -> str:
        """
        Map content types to appropriate input types for Cohere
        """
        type_mapping = {
            'document': 'search_document',
            'query': 'search_query',
            'classification': 'classification',
            'clustering': 'clustering'
        }
        return type_mapping.get(content_type, 'search_document')
    
    def batch_embed_content(self, contents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Batch embed multiple content pieces
        Each content item is a dict with 'text', 'language', and 'type' keys
        Returns list of dicts with 'embedding' added to each
        """
        texts = [item['text'] for item in contents]
        input_types = [self._get_input_type(item.get('type', 'document')) for item in contents]
        
        # Generate embeddings for all texts
        embeddings = self.embedder.generate_embeddings(texts, "search_document")  # Using search_document as default
        
        # Combine results
        results = []
        for i, item in enumerate(contents):
            result_item = item.copy()
            result_item['embedding'] = embeddings[i]
            results.append(result_item)
        
        return results

# Global instance for use throughout the application
embedding_service = MultilingualEmbeddingService()

if __name__ == "__main__":
    print("Cohere embedding service initialized")
    print("Model: embed-multilingual-v3.0")
    print("Ready for multilingual embeddings including Urdu support")