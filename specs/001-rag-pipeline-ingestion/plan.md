# Implementation Plan: RAG Pipeline - Web Scraping, Embedding Generation & Vector Storage

**Branch**: `001-rag-pipeline-ingestion` | **Date**: 2025-12-23 | **Spec**: [link to spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-pipeline-ingestion/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The RAG Pipeline implementation will create a data ingestion pipeline that crawls the deployed Docusaurus book website (https://syedsaudali-gh.github.io/Physical-AI-Humanoid-Robotics/), sitemap URL (https://syedsaudali-gh.github.io/Physical-AI-Humanoid-Robotics/sitemap.xml), extracts clean text content, chunks it appropriately (500-1000 tokens with 100-200 overlap), generates embeddings using Cohere's embed-english-v3.0 model, and stores them in Qdrant Cloud vector database with proper metadata. The implementation will be structured as a single Python project in a `/rag-backend` directory using UV for dependency management.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: cohere, qdrant-client, beautifulsoup4, requests, python-dotenv, tiktoken
**Storage**: Qdrant Cloud vector database
**Testing**: pytest
**Target Platform**: Linux server (backend service)
**Project Type**: Single project (backend service)
**Performance Goals**: Process all pages from Docusaurus book within reasonable time, handle API rate limits gracefully
**Constraints**: Must work within Qdrant Cloud Free Tier limits, handle Cohere API rate limiting, chunk size 500-1000 tokens with 100-200 overlap
**Scale/Scope**: Handle the entire Physical AI & Humanoid Robotics textbook content with metadata

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

This project must comply with the Physical AI & Humanoid Robotics Course Textbook Constitution, ensuring:
- Accuracy through verification from primary or authoritative sources
- Clarity suitable for students with AI/computer science background
- Comprehensive coverage of theoretical and practical implementations
- Integration of Qwen CLI and Spec-Kit Plus tools
- Implementation of interactivity and personalization features
- Zero plagiarism in all generated content

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-pipeline-ingestion/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
rag-backend/
├── main.py              # Main pipeline orchestrator with get_all_urls, extract_text_from_url,
│                        # chunk_text, embed, create_collection, save_chunk_to_qdrant functions
├── pyproject.toml       # Project configuration and dependencies managed by UV
├── .env                 # Environment variables (not committed to repo)
├── .gitignore           # Git ignore file
└── tests/               # Test files for the pipeline
    ├── test_scraper.py
    ├── test_text_processor.py
    └── test_embeddings.py
```

**Structure Decision**: Single project structure chosen as the RAG pipeline is a standalone data ingestion service. The implementation will be contained in a `/rag-backend` directory with a main.py file containing all required functions as specified: get_all_urls, extract_text_from_url, chunk_text, embed, create_collection named rag_embedding, save_chunk_to_qdrant, and a main function to execute the pipeline.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
