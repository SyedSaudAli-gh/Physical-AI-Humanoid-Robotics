# Data Model: RAG Chatbot - Frontend Integration with Docusaurus

## Entities

### User Query
- **Fields**:
  - id: string (unique identifier for the query)
  - content: string (the actual query text)
  - selectedText: string (optional text selected by the user)
  - timestamp: Date (when the query was created)
  - status: enum (PENDING, SENT, PROCESSING, COMPLETED, ERROR)

- **Relationships**: One-to-many with AI Response (a query can have one response)

- **Validation rules**:
  - content must be 1-1000 characters
  - selectedText can be empty or up to 5000 characters
  - timestamp must be a valid date/time

### AI Response
- **Fields**:
  - id: string (unique identifier for the response)
  - content: string (the AI-generated response text)
  - sources: array of objects (references to source documents)
  - timestamp: Date (when the response was received)
  - queryId: string (reference to the associated query)

- **Relationships**: Many-to-one with User Query (a response belongs to one query)

- **Validation rules**:
  - content must be 1-10000 characters
  - sources array can have 0-10 items
  - timestamp must be a valid date/time
  - queryId must reference an existing query

### Source Reference
- **Fields**:
  - id: string (unique identifier for the source)
  - title: string (title of the source document)
  - url: string (URL to the source document)
  - snippet: string (relevant text snippet from the source)

- **Relationships**: Belongs to AI Response

- **Validation rules**:
  - title must be 1-200 characters
  - url must be a valid URL format
  - snippet must be 1-1000 characters

## State Transitions

### Query State Transitions
- NEW → SENT: When query is successfully sent to backend
- SENT → PROCESSING: When backend acknowledges receipt
- PROCESSING → COMPLETED: When response is received from backend
- PROCESSING → ERROR: When backend returns an error
- SENT → ERROR: When network request fails

### Chat UI State Transitions
- HIDDEN → VISIBLE: When floating button is clicked
- VISIBLE → HIDDEN: When close button is clicked or user clicks outside
- COMPACT → EXPANDED: When user expands the chat window
- EXPANDED → COMPACT: When user minimizes the chat window