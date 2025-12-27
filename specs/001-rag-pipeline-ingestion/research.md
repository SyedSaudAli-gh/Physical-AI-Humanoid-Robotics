# Research for RAG Pipeline Implementation

## Decision: Project Setup and Structure
**Rationale**: The project will be created in a `/rag-backend` folder with UV (uv init) as specified in the user requirements. This provides a dedicated environment for the RAG pipeline with proper dependency management.
**Alternatives considered**: Using pip and virtualenv instead of UV; integrating directly into existing backend structure.

## Decision: Web Scraper Module
**Rationale**: Using BeautifulSoup4 and requests to crawl the Docusaurus sitemap and extract page URLs, then fetch and parse HTML content. This is a standard and well-documented approach for web scraping in Python.
**Alternatives considered**: Using Selenium for dynamic content; using crawl4ai as mentioned in the spec; using scrapy framework.

## Decision: Text Processor Module
**Rationale**: Using tiktoken for tokenization and chunking text with overlap. This ensures accurate token counts that match the embedding model's tokenization, which is important for maintaining context.
**Alternatives considered**: Using character-based chunking; using sentence-based chunking; using spaCy for more advanced NLP processing.

## Decision: Embedding & Storage Module
**Rationale**: Using Cohere's embed-english-v3.0 model for embeddings and Qdrant Cloud for vector storage with metadata. This matches the requirements in the feature specification.
**Alternatives considered**: Using OpenAI embeddings; using local embedding models like Sentence Transformers; using other vector databases like Pinecone or Weaviate.

## Decision: Pipeline Orchestrator
**Rationale**: Creating a main script (main.py) that orchestrates the full ingestion flow: scrape → process → embed → store. This provides a clear, single entry point for the pipeline.
**Alternatives considered**: Using workflow tools like Airflow or Prefect; breaking into separate scripts; using a more complex microservice architecture.

## Decision: Deployment URL
**Rationale**: Using the provided deployment link (https://syedsaudali-gh.github.io/Physical-AI-Humanoid-Robotics/) as the source for the Docusaurus book content.
**Alternatives considered**: Using a local copy of the content; using a different deployment URL.

## Decision: Main.py System Design
**Rationale**: Implementing specific functions as requested: get_all_urls, extract_text_from_url, chunk_text, embed, create_collection named rag_embedding, save_chunk_to_qdrant, and execute in main function. This provides a clear, modular approach to the pipeline.
**Alternatives considered**: Using different function names or a different architecture pattern.

## Decision: Idempotent Operation
**Rationale**: Implementing idempotent behavior by checking for existing entries before creating new ones, ensuring re-runs don't create duplicates.
**Alternatives considered**: Using UUIDs based on content to prevent duplicates; using timestamps to track runs; implementing a more complex state management system.