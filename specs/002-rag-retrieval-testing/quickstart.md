# Quickstart Guide for RAG Pipeline Retrieval

## Prerequisites

- Python 3.11+
- Access to Cohere API (with embed-english-v3.0 model access)
- Access to Qdrant Cloud with the 'rag_embedding' collection created in Spec 1
- Completed ingestion pipeline from Spec 1 (content must already be stored in Qdrant)

## Setup

1. Ensure you have the rag-backend directory from Spec 1:
   ```bash
   cd rag-backend
   ```

2. Install dependencies (if not already installed from Spec 1):
   ```bash
   uv pip install cohere qdrant-client python-dotenv pytest
   ```

3. Make sure your `.env` file contains the required API keys:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

## Running the Retrieval Service

1. To perform a retrieval query, you can use the retrieval service directly:
   ```python
   from retrieval_service import retrieve
   
   results = retrieve("your query text here", top_k=5)
   print(results)
   ```

2. Or run the retrieval from the command line if implemented in main.py:
   ```bash
   python main.py --query "your query text here" --top-k 5
   ```

## Running Tests

1. To run unit tests for retrieval functionality:
   ```bash
   pytest test_retrieval.py -v
   ```

2. To run integration tests for the full pipeline:
   ```bash
   pytest test_pipeline.py -v
   ```

3. To run all tests:
   ```bash
   pytest
   ```

## Configuration

The retrieval service can be configured by modifying the Query Parameters:
- Top-k: Number of results to return (default: 5)
- Response timeout: Maximum time to wait for response (default: 2000ms)
- Minimum score threshold: Minimum similarity score for inclusion (default: 0.0)

## Example Usage

```python
from retrieval_service import retrieve

# Basic retrieval
results = retrieve("What is ROS 2?", top_k=5)

# Retrieval with custom parameters
results = retrieve("How does Qdrant work?", top_k=10, min_score_threshold=0.3)

# Check results
for chunk in results.retrieved_chunks:
    print(f"Score: {chunk.similarity_score}")
    print(f"Content: {chunk.content[:200]}...")  # First 200 chars
    print(f"Source: {chunk.source_url}")
    print("---")
```

## Troubleshooting

- If queries return no results, verify that the ingestion pipeline from Spec 1 completed successfully and content exists in Qdrant
- If response times are too slow, check your Cohere and Qdrant API connection speeds
- For authentication errors, verify your API keys in the .env file