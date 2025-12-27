"""
Test script to check Qdrant query_points response structure
"""
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=10
)

# Test the structure of query_points response
try:
    # Get the collection info
    collection_info = qdrant_client.get_collection("physical_ai_docs")
    print("Collection info:", collection_info)
    
    # Test query with a simple vector (using mock embedding for a test string)
    from mock_embedding import mock_embed_text
    test_vector = mock_embed_text("test query")
    
    print(f"Test vector length: {len(test_vector)}")
    
    # Perform a test query
    results = qdrant_client.query_points(
        collection_name="physical_ai_docs",
        query=test_vector,
        limit=2,
        with_payload=True
    )
    
    print("Query results type:", type(results))
    print("Query results:", results)
    
    # Check the structure of the first result if any
    if results:
        first_result = results[0] if hasattr(results, '__getitem__') else results.points[0]
        print("First result type:", type(first_result))
        print("First result:", first_result)
        print("Available attributes:", [attr for attr in dir(first_result) if not attr.startswith('_')])
        
        # Check if it has payload and other attributes
        if hasattr(first_result, 'payload'):
            print("Payload:", first_result.payload)
        if hasattr(first_result, 'score'):
            print("Score:", first_result.score)
        if hasattr(first_result, 'id'):
            print("ID:", first_result.id)
        if hasattr(first_result, 'vector'):
            print("Vector (first 5 elements):", first_result.vector[:5] if first_result.vector else None)
            
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()