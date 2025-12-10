# Research: Physical AI & Humanoid Robotics Textbook

## Decision: Embedding provider & model
**Rationale:** Cohere embed-multilingual-v3.0 provides better support for Urdu translation while maintaining high quality for English content, which is essential for our multilingual textbook feature.
**Alternatives considered:** 
- Cohere embed-english-v3.0: Higher accuracy for English but no multilingual support
- OpenAI embeddings: Good quality but no specific multilingual advantage
- Hugging Face models: Free but require more infrastructure management

## Decision: Vector database chunking strategy  
**Rationale:** 512-token chunks with 128 overlap provides better precision for RAG responses while keeping costs manageable, ensuring the chatbot gives accurate answers based on context.
**Alternatives considered:**
- 1024-token chunks: Higher cost, potential for less relevant information in responses
- 256-token chunks: Higher precision but might miss important context relationships
- Character-based chunks: Less semantic coherence

## Decision: Personalization levels
**Rationale:** 3 tiers (Beginner/Intermediate/Advanced) based on specific technical skills provides clear user segmentation while remaining manageable from an implementation standpoint.
**Alternatives considered:**
- Free-form questionnaire responses: More granular but harder to implement personalization logic
- 5-tier system: More detailed but potentially confusing for users
- Binary classification: Too simplistic for diverse technical backgrounds

## Decision: Urdu translation method
**Rationale:** Qwen 1.5 Pro via subagent balances technical term accuracy with reasonable cost, and integrates with our required Qwen CLI ecosystem.
**Alternatives considered:**
- Google Translate API: Good quality but doesn't meet requirement to showcase Qwen CLI
- DeepL: High quality but more expensive and doesn't showcase Qwen CLI
- Manual translation: Guaranteed accuracy but too time-intensive for hackathon

## Decision: Hosting for FastAPI backend
**Rationale:** Render.com free tier offers a good balance of reliability and cost for the hackathon timeframe, with sufficient resources for our expected load.
**Alternatives considered:**
- Railway: Similar free tier but slightly more complex deployment
- Fly.io: Good performance but potential cold start issues
- Self-hosting: Full control but requires more maintenance during hackathon

## Technology Research Findings

### Docusaurus Implementation
- Docusaurus 3.0+ supports custom React components for interactive features
- Can embed forms and interactive elements for personalization
- Supports client-side language translation via plugins

### RAG Architecture with Cohere & Qdrant
- Cohere's embed-multilingual-v3.0 supports 100+ languages including Urdu
- Qdrant Cloud offers free tier with sufficient capacity for hackathon
- RAG pipeline: Document chunking → Embedding generation → Storage in Qdrant → Query-time embedding + similarity search

### Better-Auth Integration
- Supports custom fields for user background questionnaire
- Provides JWT-based authentication compatible with Docusaurus
- Offers various authentication methods (email, OAuth providers)

### Qwen CLI Subagents for Content Generation
- Can be configured to generate content following specific formatting guidelines
- Supports code example generation with validation against official docs
- Can handle multilingual translation tasks
- Can generate Mermaid diagrams for technical concepts

### Performance & Scalability Considerations
- Docusaurus sites deployed to GitHub Pages offer excellent performance
- RAG responses will be cached to reduce Cohere API calls
- Personalization will be implemented through client-side content swapping
- CDN distribution through GitHub Pages for static assets

### Quality Validation Strategy
- Code examples will be tested in Docker containers to ensure correctness
- Automated readability checks using Flesch-Kincaid calculations
- Manual spot-checks for Urdu translation accuracy
- Automated testing of RAG accuracy using predefined question sets

### Deployment Architecture
- Frontend: Docusaurus site deployed to GitHub Pages (static hosting)
- Backend: FastAPI app on Render.com (dynamic features like RAG, auth)
- Database: Neon Postgres for user data and configuration
- Vector DB: Qdrant Cloud for embeddings and RAG
- Authentication: Better-Auth with PostgreSQL adapter