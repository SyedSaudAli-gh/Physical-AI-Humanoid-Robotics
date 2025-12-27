# API Contract: Chat Endpoint

## POST /api/chat

### Description
Endpoint to send user queries to the backend and receive AI-generated responses with source references.

### Request
- **Method**: POST
- **URL**: /api/chat
- **Content-Type**: application/json

#### Request Body
```json
{
  "query": {
    "type": "string",
    "description": "The user's query text",
    "example": "How do I set up ROS 2 for my humanoid robot?",
    "minLength": 1,
    "maxLength": 1000
  },
  "selected_text": {
    "type": "string",
    "description": "Optional text selected by the user to provide context",
    "example": "The URDF file defines the robot's physical properties including links, joints, and materials.",
    "maxLength": 5000,
    "nullable": true
  }
}
```

### Response
- **Success Response**: 200 OK
- **Error Response**: 400 Bad Request, 500 Internal Server Error

#### Success Response Body
```json
{
  "response": {
    "type": "string",
    "description": "The AI-generated response",
    "example": "To set up ROS 2 for your humanoid robot, you'll need to install the appropriate packages and configure your URDF files..."
  },
  "sources": {
    "type": "array",
    "description": "List of source documents referenced in the response",
    "items": {
      "type": "object",
      "properties": {
        "title": {
          "type": "string",
          "description": "Title of the source document",
          "example": "ROS 2 Installation Guide"
        },
        "url": {
          "type": "string",
          "description": "URL to the source document",
          "format": "uri",
          "example": "https://docs.ros.org/en/humble/Installation.html"
        },
        "snippet": {
          "type": "string",
          "description": "Relevant text snippet from the source",
          "example": "ROS 2 is a set of software libraries and tools that help you build robot applications. It provides libraries and tools to create robot applications."
        }
      }
    },
    "maxItems": 10
  }
}
```

#### Error Response Body
```json
{
  "error": {
    "type": "string",
    "description": "Error message describing what went wrong",
    "example": "Query too long. Maximum 1000 characters allowed."
  }
}
```

### Example Request
```http
POST /api/chat
Content-Type: application/json

{
  "query": "How do I configure URDF for my humanoid robot?",
  "selected_text": "The URDF file defines the robot's physical properties including links, joints, and materials."
}
```

### Example Response
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "response": "To configure URDF for your humanoid robot, you need to define the kinematic and dynamic properties of each link and joint. Make sure to specify the correct parent-child relationships between joints...",
  "sources": [
    {
      "title": "URDF Robot Description Format",
      "url": "http://wiki.ros.org/urdf",
      "snippet": "URDF is a robot description format that allows users to describe their robots in a unified way. It defines the kinematic and dynamic properties of robots."
    },
    {
      "title": "Building Your First Robot with ROS 2",
      "url": "https://docs.ros.org/en/humble/Tutorials/URDF/Building-A-Robot.html",
      "snippet": "In this tutorial, you will learn how to build a URDF file for a simple robot, including defining links, joints, and materials."
    }
  ]
}
```