# Internal API Contracts for RAG Pipeline

## Function: get_all_urls
- **Purpose**: Crawls the Docusaurus sitemap to extract all page URLs
- **Input**: base_url (string) - The base URL of the Docusaurus book
- **Output**: List of URLs (array of strings)
- **Errors**: NetworkError if the sitemap cannot be accessed
- **Side Effects**: None

## Function: extract_text_from_url
- **Purpose**: Fetches and parses HTML content from a URL, extracting clean text
- **Input**: url (string) - The URL to fetch content from
- **Output**: Dictionary with 'title' (string) and 'text' (string) fields
- **Errors**: HTTPError if the page cannot be fetched; ParseError if HTML cannot be parsed
- **Side Effects**: Makes HTTP request to the provided URL

## Function: chunk_text
- **Purpose**: Splits text into chunks of specified size with overlap
- **Input**: 
  - text (string) - The text to be chunked
  - chunk_size_min (integer) - Minimum tokens per chunk (default: 500)
  - chunk_size_max (integer) - Maximum tokens per chunk (default: 1000)
  - overlap_size (integer) - Token overlap between chunks (default: 150)
- **Output**: List of text chunks (array of strings)
- **Errors**: ValueError if chunk sizes are invalid
- **Side Effects**: None

## Function: embed
- **Purpose**: Generates embeddings for text chunks using Cohere API
- **Input**: texts (array of strings) - The text chunks to embed
- **Output**: Array of embedding vectors (array of float arrays)
- **Errors**: APIError if Cohere API call fails; RateLimitError if rate limits are exceeded
- **Side Effects**: Makes API call to Cohere service

## Function: create_collection
- **Purpose**: Creates or verifies the existence of the rag_embedding collection in Qdrant
- **Input**: collection_name (string) - Name of the collection (default: 'rag_embedding')
- **Output**: Boolean indicating success
- **Errors**: DatabaseError if collection creation fails
- **Side Effects**: Creates or verifies collection in Qdrant Cloud

## Function: save_chunk_to_qdrant
- **Purpose**: Saves a text chunk with its embedding and metadata to Qdrant
- **Input**:
  - chunk_text (string) - The text of the chunk
  - embedding (array of floats) - The embedding vector
  - metadata (object) - Metadata including source_url, page_title, chunk_id, timestamp
- **Output**: Boolean indicating success
- **Errors**: DatabaseError if saving to Qdrant fails
- **Side Effects**: Adds record to Qdrant Cloud collection

## Function: main
- **Purpose**: Orchestrates the full ingestion pipeline: scrape → process → embed → store
- **Input**: None (uses configuration constants)
- **Output**: None (writes results to Qdrant Cloud)
- **Errors**: Various errors from the component functions
- **Side Effects**: Processes the entire Docusaurus book and stores embeddings in Qdrant Cloud