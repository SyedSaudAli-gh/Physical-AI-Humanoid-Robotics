# Data Model for RAG Pipeline Retrieval

## Entity: Query
- **Description**: Represents a user's text input that needs to be matched against stored content
- **Fields**:
  - query_text: string (the original text query from the user)
  - query_embedding: float array (vector representation of the query)
  - timestamp: datetime (when the query was processed)
  - top_k: integer (number of results to return, default 5)

## Entity: Retrieved Chunk
- **Description**: A text chunk from the knowledge base that matches the query, with similarity score and metadata
- **Fields**:
  - chunk_id: string (unique identifier for the chunk)
  - content: string (the actual text content of the chunk)
  - similarity_score: float (cosine similarity score between query and chunk)
  - source_url: string (URL of the original webpage)
  - page_title: string (title of the original webpage)
  - chunk_metadata: object (additional metadata stored with the chunk)
  - rank: integer (position in the ranked results list)

## Entity: Search Result
- **Description**: A collection of retrieved chunks with their scores and metadata, returned in response to a query
- **Fields**:
  - query_id: string (identifier for the original query)
  - retrieved_chunks: array of Retrieved Chunk (list of matching chunks)
  - total_results: integer (total number of matching chunks found)
  - search_time_ms: float (time taken to perform the search)
  - query_text: string (the original query text for reference)

## Entity: Query Parameters
- **Description**: Configuration parameters for the retrieval process
- **Fields**:
  - top_k: integer (number of results to return, default 5)
  - response_timeout_ms: integer (maximum time to wait for response, default 2000ms)
  - min_score_threshold: float (minimum similarity score for inclusion, default 0.0)
  - collection_name: string (name of the Qdrant collection to search, default 'rag_embedding')

## Relationships
- One Query can result in many Retrieved Chunks (1 to many)
- Many Retrieved Chunks belong to one Search Result (many to 1)
- One Query has one set of Query Parameters (1 to 1)

## Validation Rules
- Query text must not be empty or contain only whitespace
- Top-k parameter must be between 1 and 100
- Response time must be under 2 seconds
- Similarity scores must be between 0 and 1
- Retrieved chunks must include all required metadata fields