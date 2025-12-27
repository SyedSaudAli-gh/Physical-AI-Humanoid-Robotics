# API Contract: Chatbot Integration

## POST /api/chat

### Description
Endpoint for the chatbot widget to send user queries and receive AI-generated responses with source references.

### Request
```
POST /api/chat
Content-Type: application/json
```

#### Request Body
```json
{
  "message": "string, the user's query message",
  "context": "string, optional selected text context from the page"
}
```

#### Request Body Properties
- **message** (required): The query or message from the user
  - Type: string
  - Max length: 2000 characters
  - Example: "Explain how ROS 2 nodes communicate with each other"

- **context** (optional): Selected text that provides context for the query
  - Type: string
  - Max length: 5000 characters
  - Example: "ROS 2 uses a publish-subscribe pattern for communication between nodes..."

### Response
#### Success Response (200 OK)
```json
{
  "response": "string, the AI-generated response",
  "sources": [
    {
      "title": "string, title of the source document",
      "url": "string, URL to the source",
      "excerpt": "string, relevant excerpt from the source"
    }
  ],
  "timestamp": "string, ISO 8601 formatted timestamp"
}
```

#### Error Response (400 Bad Request)
```json
{
  "error": "string, description of the error",
  "code": "string, error code"
}
```

#### Error Response (500 Internal Server Error)
```json
{
  "error": "string, description of the error",
  "code": "string, error code"
}
```

### Example Request
```json
{
  "message": "How do I create a publisher in ROS 2?",
  "context": "In ROS 2, nodes communicate using a publish-subscribe pattern..."
}
```

### Example Response
```json
{
  "response": "To create a publisher in ROS 2, you need to initialize a node, create a publisher with a specific topic name and message type, and then publish messages using the publisher object. Here's a basic example in Python...",
  "sources": [
    {
      "title": "ROS 2 Publisher Tutorial",
      "url": "https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html",
      "excerpt": "Creating a publisher in ROS 2 involves initializing a node, creating a publisher with a topic name and message type..."
    }
  ],
  "timestamp": "2025-12-24T10:30:00Z"
}
```

### Headers
- **Content-Type**: application/json
- **Accept**: application/json

### Authentication
This endpoint does not require authentication for basic functionality but may include optional session tracking headers for analytics.