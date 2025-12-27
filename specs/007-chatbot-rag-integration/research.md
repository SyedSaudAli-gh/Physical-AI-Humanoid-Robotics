# Research Findings: Docusaurus Chatbot ↔ RAG Backend Integration

## Current Architecture

### Frontend (Docusaurus)
- **Technology Stack**: React 18, Docusaurus 3.0.0, JavaScript/TypeScript
- **Node Version**: >=18.0 (as per package.json engines field)
- **Dependencies**: React, Docusaurus, Axios, Bootstrap, styled-components
- **Testing Framework**: Jest, React Testing Library, Cypress
- **API Communication**: Fetch API with timeout handling
- **Current Chatbot Component**: Located in `src/components/ChatWidget/` with:
  - Floating button and expandable window
  - Text selection support
  - Response display with sources
  - Error handling

### Backend (FastAPI RAG)
- **Technology Stack**: Python 3.9+, FastAPI, uvicorn
- **Dependencies**: fastapi>=0.104.0, uvicorn>=0.24.0, qdrant-client, openai, google-generativeai
- **Testing Framework**: pytest
- **API Endpoints**:
  - POST `/api/chat` - for chat queries
  - GET `/health` - for health checks
- **Data Models**: Pydantic models for request/response validation
- **CORS Configuration**: Currently allows all origins (in development)

## API Contract Details

### Request Format
- **Endpoint**: POST `/api/chat`
- **Headers**: `Content-Type: application/json`
- **Body**: 
  ```json
  {
    "query": "user's question",
    "selected_text": "optional selected text for context"
  }
  ```

### Response Format
- **Success Response**:
  ```json
  {
    "answer": "AI-generated response",
    "sources": [
      {
        "source_url": "URL of source",
        "page_title": "Title of source page",
        "snippet": "Relevant snippet from source",
        "relevance_score": 0.8
      }
    ]
  }
  ```

## Identified Integration Points

### Frontend API Service
- Located in `src/services/api.js`
- Uses `API_ENDPOINTS.CHAT` constant which maps to `/api/chat`
- Already has error handling and timeout functionality
- Handles response validation

### CORS Configuration
- Backend already configured with CORS middleware
- Currently allows all origins (needs to be restricted in production)

## Performance Requirements

Based on the feature specification:
- Response time: < 5 seconds for 95% of requests
- Uptime: 99% during standard operating hours
- Error rate: < 1% for API communication
- Success rate: 90% of queries result in helpful responses

## Key Decisions

### 1. Communication Protocol
- Decision: Use HTTP POST requests with JSON payload
- Rationale: Standard REST approach, already implemented in both frontend and backend
- Alternatives considered: WebSocket for real-time communication (rejected due to complexity for basic chat functionality)

### 2. Error Handling Strategy
- Decision: Implement client-side error handling with user-friendly messages
- Rationale: Provides better user experience when backend is unavailable
- Alternatives considered: Simple error alerts (rejected for poor UX)

### 3. Timeout Configuration
- Decision: Set 30-second timeout for API requests
- Rationale: Balances between giving backend sufficient time to process complex queries and preventing UI hanging
- Alternatives considered: Shorter (10s) or longer (60s) timeouts (settled on 30s as optimal)

### 4. CORS Policy
- Decision: Currently allow all origins for development, restrict in production
- Rationale: Enables local development flexibility while acknowledging security concerns
- Alternatives considered: Restrictive policy from start (rejected as it would complicate development)

## Security Considerations

- Authentication: Currently not implemented, may need to be added for production
- Input validation: Implemented on both frontend and backend
- Rate limiting: Not currently implemented, may need to be added for production