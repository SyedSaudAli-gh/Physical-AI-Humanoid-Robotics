# API Contract: Docusaurus Chatbot ↔ RAG Backend Integration

## Overview
This document specifies the API contract between the Docusaurus frontend chatbot and the RAG backend service. The API follows REST principles with JSON payloads for request and response bodies.

## Base URL
- Development: `http://127.0.0.1:8000`
- Production: `[To be determined]`

## Endpoints

### POST /api/chat
Submit a user query to the RAG backend for processing and receive an AI-generated response.

#### Request
- **Method**: POST
- **Path**: `/api/chat`
- **Content-Type**: `application/json`
- **Headers**:
  - `Content-Type: application/json`

#### Request Body
```json
{
  "query": "string (required) - The user's question",
  "selected_text": "string (optional) - Text selected by the user for context",
  "context": "object (optional) - Additional context like module/chapter info"
}
```

**Validation**:
- `query` must be a non-empty string with minimum length of 1 character
- `selected_text` if provided, must be a string with maximum length of 1000 characters
- `context` if provided, must be a valid JSON object

#### Response
- **Success Response**: `200 OK`
- **Content-Type**: `application/json`

```json
{
  "answer": "string (required) - The AI-generated response to the user's query",
  "sources": [
    {
      "source_url": "string (required) - URL of the original source",
      "page_title": "string (required) - Title of the original page",
      "snippet": "string (required) - Relevant snippet from the source",
      "relevance_score": "number (required) - Relevance score (0.0 to 1.0)"
    }
  ]
}
```

**Validation**:
- `answer` must be a non-empty string with minimum length of 1 character
- `sources` if provided, must be an array of valid source reference objects
- Each source object must have valid values for all required fields

#### Error Responses
- **400 Bad Request**: Request body validation failed
  - Response body: `{"detail": "Validation error details"}`
- **422 Unprocessable Entity**: Request validation failed
  - Response body: `{"detail": "Validation error details"}`
- **500 Internal Server Error**: Server-side error occurred
  - Response body: `{"detail": "Internal server error details"}`

### GET /health
Check the health status of the RAG backend service.

#### Request
- **Method**: GET
- **Path**: `/health`

#### Response
- **Success Response**: `200 OK`
- **Content-Type**: `application/json`

```json
{
  "status": "string - Health status of the service",
  "service": "string - Name of the service",
  "version": "string - Version of the service"
}
```

## Authentication
Currently no authentication required. In production, authentication headers may be required.

## Rate Limiting
Currently no rate limiting implemented. May be added in future releases.

## CORS Policy
The backend supports CORS requests from frontend origins. Currently configured to allow all origins for development purposes.