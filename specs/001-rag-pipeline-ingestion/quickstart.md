# Quickstart Guide for RAG Pipeline

## Prerequisites

- Python 3.11+
- UV package manager
- Access to Cohere API (with embed-english-v3.0 model access)
- Qdrant Cloud account and API key
- Access to the Docusaurus book at https://syedsaudali-gh.github.io/Physical-AI-Humanoid-Robotics/

## Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Navigate to the rag-backend directory:
   ```bash
   cd rag-backend
   ```

3. Initialize the project with UV:
   ```bash
   uv init
   ```

4. Install dependencies:
   ```bash
   uv pip install cohere qdrant-client beautifulsoup4 requests python-dotenv tiktoken
   ```

5. Create a `.env` file in the project root with your API keys:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

## Configuration

The pipeline can be configured by modifying the following constants in main.py:
- CHUNK_SIZE_MIN: Minimum tokens per chunk (default: 500)
- CHUNK_SIZE_MAX: Maximum tokens per chunk (default: 1000)
- OVERLAP_SIZE: Token overlap between chunks (default: 150)
- BASE_URL: The Docusaurus book URL (default: https://syedsaudali-gh.github.io/Physical-AI-Humanoid-Robotics/)

## Running the Pipeline

1. Execute the main script:
   ```bash
   python main.py
   ```

2. The pipeline will:
   - Crawl all pages from the Docusaurus book
   - Extract and clean text content
   - Chunk the text with appropriate overlap
   - Generate embeddings using Cohere
   - Store embeddings in Qdrant Cloud with metadata

## Monitoring

- Progress and errors are logged to the console
- Check the logs for any failed pages that might need reprocessing
- Verify the 'rag_embedding' collection exists in your Qdrant Cloud instance

## Troubleshooting

- If you encounter rate limit errors from Cohere, the pipeline will automatically retry with exponential backoff
- If Qdrant Cloud is unavailable, the pipeline will pause and retry
- For large sites, consider running during off-peak hours to avoid rate limiting