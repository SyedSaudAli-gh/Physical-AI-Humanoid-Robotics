# API Contracts: Physical AI & Humanoid Robotics Textbook

## Authentication API

### POST /api/auth/signup
Register a new user with background questionnaire
- **Request Body:**
  ```json
  {
    "email": "string (required)",
    "password": "string (required)",
    "name": "string (required)",
    "technical_skills": ["string (optional)"],
    "experience_level": "enum (optional)",
    "background_questionnaire": {
      "years_experience": "number (optional)",
      "primary_language": "string (optional)",
      "educational_background": "string (optional)",
      "hardware_access": "boolean (optional)"
    }
  }
  ```
- **Response:**
  ```json
  {
    "user_id": "string",
    "email": "string",
    "name": "string",
    "auth_token": "JWT string",
    "created_at": "timestamp"
  }
  ```

### POST /api/auth/login
Login existing user
- **Request Body:**
  ```json
  {
    "email": "string (required)",
    "password": "string (required)"
  }
  ```
- **Response:**
  ```json
  {
    "user_id": "string",
    "auth_token": "JWT string"
  }
  ```

## Textbook Content API

### GET /api/modules
Retrieve all textbook modules
- **Response:**
  ```json
  [
    {
      "id": "string",
      "name": "string",
      "description": "string",
      "order_index": "number",
      "learning_objectives": ["string"],
      "prerequisites": ["string"]
    }
  ]
  ```

### GET /api/modules/{module_id}/chapters
Retrieve chapters for a specific module
- **Response:**
  ```json
  [
    {
      "id": "string",
      "title": "string",
      "order_index": "number",
      "learning_outcomes": ["string"],
      "content_preview": "string"
    }
  ]
  ```

### GET /api/chapters/{chapter_id}
Retrieve specific chapter content (with personalization applied)
- **Query Parameters:**
  - difficulty (optional): "beginner", "intermediate", "advanced"
  - language (optional): "en", "ur", etc.
- **Headers:**
  - Authorization: "Bearer {auth_token}" (optional)
- **Response:**
  ```json
  {
    "id": "string",
    "title": "string",
    "content": "string",
    "module_id": "string",
    "order_index": "number",
    "learning_outcomes": ["string"],
    "code_examples": [
      {
        "language": "string",
        "code": "string",
        "description": "string"
      }
    ],
    "exercises": ["exercise objects"]
  }
  ```

## RAG Chatbot API

### POST /api/chat/query
Query the RAG system with book context
- **Headers:**
  - Authorization: "Bearer {auth_token}" (optional)
- **Request Body:**
  ```json
  {
    "query": "string (required)",
    "selected_text": "string (optional)",
    "chapter_id": "string (optional)",
    "include_context": "boolean (default: true)"
  }
  ```
- **Response:**
  ```json
  {
    "query": "string",
    "response": "string",
    "sources": [
      {
        "chapter_id": "string",
        "chapter_title": "string",
        "relevance_score": "number"
      }
    ],
    "timestamp": "timestamp"
  }
  ```

## User Preferences API

### PUT /api/users/preferences
Update user preferences including personalization settings
- **Headers:**
  - Authorization: "Bearer {auth_token}"
- **Request Body:**
  ```json
  {
    "preferred_language": "string (optional)",
    "chapter_difficulty_override": "enum (optional)",
    "notification_preferences": "object (optional)"
  }
  ```
- **Response:**
  ```json
  {
    "user_id": "string",
    "preferences": {
      "preferred_language": "string",
      "chapter_difficulty_override": "enum",
      "notification_preferences": "object"
    },
    "updated_at": "timestamp"
  }
  ```

## Translation API

### POST /api/translate
Translate content to specified language
- **Headers:**
  - Authorization: "Bearer {auth_token}" (optional)
- **Request Body:**
  ```json
  {
    "content": "string (required)",
    "target_language": "string (required)",
    "source_language": "string (optional, default: en)"
  }
  ```
- **Response:**
  ```json
  {
    "original_content": "string",
    "translated_content": "string",
    "target_language": "string",
    "source_language": "string"
  }
  ```