# Data Model for RAG Pipeline

## Entity: Document Chunk
- **Description**: Represents a segment of text content from the Docusaurus book with associated embedding and metadata
- **Fields**:
  - id: string (unique identifier for the chunk)
  - content: string (the actual text content of the chunk)
  - embedding: float array (vector representation of the text content)
  - source_url: string (URL of the original webpage)
  - page_title: string (title of the original webpage)
  - chunk_id: string (index of the chunk within the page)
  - timestamp: datetime (when the chunk was created/processed)
  - token_count: integer (number of tokens in the chunk)

## Entity: Crawled Page
- **Description**: Represents a webpage from the Docusaurus book that has been crawled
- **Fields**:
  - url: string (URL of the webpage)
  - title: string (title of the webpage)
  - raw_html: string (raw HTML content of the page)
  - processing_status: string (status of processing: 'pending', 'processed', 'failed')
  - last_crawled: datetime (timestamp of last crawl)
  - content_hash: string (hash of content to detect changes)

## Entity: Embedding
- **Description**: Represents the vector representation of a text chunk
- **Fields**:
  - chunk_id: string (reference to the document chunk)
  - vector: float array (the embedding vector)
  - model_used: string (name of the embedding model used)
  - created_at: datetime (timestamp when embedding was created)

## Entity: Processing Run
- **Description**: Represents a single execution of the ingestion pipeline
- **Fields**:
  - run_id: string (unique identifier for the run)
  - start_time: datetime (when the run started)
  - end_time: datetime (when the run ended)
  - status: string (status of the run: 'running', 'completed', 'failed')
  - processed_pages: integer (number of pages processed)
  - created_chunks: integer (number of chunks created)
  - errors: array (list of errors encountered during the run)

## Relationships
- One Crawled Page can have many Document Chunks (1 to many)
- One Document Chunk has one Embedding (1 to 1)
- One Processing Run can create many Document Chunks (1 to many)
- One Processing Run can process many Crawled Pages (1 to many)

## Validation Rules
- Document Chunk content must be between 500-1000 tokens
- Document Chunk must have valid source_url and page_title
- Embedding vector must match the expected dimensions for the model used
- Chunk overlap must be between 100-200 tokens
- Source URL must be from the specified Docusaurus book domain