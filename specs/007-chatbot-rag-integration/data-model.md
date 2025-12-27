# Data Model: Docusaurus Chatbot ↔ RAG Backend Integration

## Entities

### User Query
- **Description**: A text-based question or request submitted by the user through the chatbot interface
- **Fields**:
  - `query`: string (required) - The main question from the user
  - `selected_text`: string (optional) - Optional text that the user has highlighted or selected for context
  - `context`: object (optional) - Additional context like module/chapter information
- **Validation**:
  - `query` must be a non-empty string with minimum length of 1 character
  - `selected_text` if provided, must be a string with maximum length of 1000 characters
- **Relationships**: Sent from frontend to backend via API request

### AI Response
- **Description**: The generated text response created by the RAG system based on the user's query and available documentation
- **Fields**:
  - `answer`: string (required) - The agent's response to the user's query
  - `sources`: array of SourceReference (optional) - List of sources used to generate the answer
- **Validation**:
  - `answer` must be a non-empty string with minimum length of 1 character
  - `sources` if provided, must be an array of valid SourceReference objects
- **Relationships**: Returned from backend to frontend via API response

### SourceReference
- **Description**: Represents a reference to the original content that contributed to the agent's answer
- **Fields**:
  - `source_url`: string (required) - URL of the original source
  - `page_title`: string (required) - Title of the original page
  - `snippet`: string (required) - Relevant snippet from the source
  - `relevance_score`: float (required) - Relevance score of this source to the query (0.0 to 1.0)
- **Validation**:
  - `source_url` must be a valid URL string
  - `page_title` and `snippet` must be non-empty strings
  - `relevance_score` must be between 0.0 and 1.0
- **Relationships**: Part of AI Response, represents the sources used in generating the response

### ChatMessage
- **Description**: Represents a single message in the chat conversation
- **Fields**:
  - `id`: string (required) - Unique identifier for the message
  - `sender`: enum (required) - Identifies the message sender ('user', 'assistant', 'system')
  - `content`: string (required) - The text content of the message
  - `timestamp`: Date (required) - When the message was created/sent
  - `sources`: array of string (optional) - Source references for AI-generated responses
  - `isError`: boolean (optional) - Indicates if this is an error message
- **Validation**:
  - `id` must be a unique string
  - `sender` must be one of the allowed values
  - `content` must be a non-empty string
  - `timestamp` must be a valid Date object
- **Relationships**: Part of Chat Session, represents individual messages in the conversation

### Chat Session
- **Description**: A sequence of related interactions between a user and the chatbot that may span multiple queries and responses
- **Fields**:
  - `sessionId`: string (required) - Unique identifier for the session
  - `messages`: array of ChatMessage (required) - List of messages in the session
  - `createdAt`: Date (required) - When the session was created
  - `lastActiveAt`: Date (required) - When the session was last active
- **Validation**:
  - `sessionId` must be a unique string
  - `messages` must be an array of valid ChatMessage objects
  - `createdAt` and `lastActiveAt` must be valid Date objects
- **Relationships**: Contains multiple ChatMessage entities, represents the conversation flow