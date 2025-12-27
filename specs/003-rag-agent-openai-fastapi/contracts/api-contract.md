# API Contract for RAG Agent

## Endpoint: POST /api/chat

### Description
Accepts user questions and returns AI-generated answers based on the textbook content with source references.

### Request
- **Method**: POST
- **Path**: /api/chat
- **Content-Type**: application/json

#### Request Body
```json
{
  "query": "string (required) - The user's question about the textbook content",
  "selected_text": "string (optional) - Text that the user has highlighted or selected for context"
}
```

**Example Request**:
```json
{
  "query": "Explain how ROS 2 nodes communicate?",
  "selected_text": "nodes and topics"
}
```

### Response
- **Success Response**: 200 OK
- **Content-Type**: application/json

#### Response Body
```json
{
  "answer": "string - The AI-generated answer to the user's question",
  "sources": [
    {
      "source_url": "string - URL of the original source",
      "page_title": "string - Title of the original page",
      "snippet": "string - Relevant snippet from the source",
      "relevance_score": "float - Relevance score of this source to the query"
    }
  ]
}
```

**Example Response**:
```json
{
  "answer": "ROS 2 nodes communicate through topics, services, and actions. Topics provide a publish-subscribe communication model...",
  "sources": [
    {
      "source_url": "https://syedsaudali-gh.github.io/Physical-AI-Humanoid-Robotics/ros2/communication",
      "page_title": "Communication in ROS 2",
      "snippet": "ROS 2 nodes communicate through topics, services, and actions. Topics provide a publish-subscribe communication model where nodes can publish messages to topics and subscribe to topics to receive messages...",
      "relevance_score": 0.95
    }
  ]
}
```

### Error Responses
- **422 Unprocessable Entity**: When request body validation fails
- **500 Internal Server Error**: When there's an internal server error (e.g., OpenAI API failure, retrieval service unavailable)

## Validation Rules
- query field is required and must not be empty
- selected_text field is optional
- answer field in response must not be empty
- sources field in response must be an array of source references