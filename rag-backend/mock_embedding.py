"""
Mock embedding service for testing when API quotas are exceeded
"""
import hashlib
from typing import List
import numpy as np

def mock_embed_text(text: str) -> List[float]:
    """
    Create a deterministic mock embedding for text
    This is for testing purposes when API quotas are exceeded
    """
    # Create a hash of the text to generate consistent mock embeddings
    text_hash = hashlib.md5(text.encode()).hexdigest()
    
    # Convert hex hash to a list of floats in the appropriate range
    # Using numpy to create an array with the proper dimensions (768 for Google's embedding-001)
    embedding = []
    for i in range(0, len(text_hash), 2):
        if i + 1 < len(text_hash):
            # Take two hex characters and convert to a value between -1 and 1
            hex_pair = text_hash[i:i+2]
            value = int(hex_pair, 16) / 128.0 - 1.0  # Normalize to [-1, 1] range
            embedding.append(value)
    
    # Pad or truncate to 768 dimensions (Google's embedding-001 size)
    while len(embedding) < 768:
        embedding.append(0.0)
    
    return embedding[:768]  # Truncate to exactly 768 dimensions