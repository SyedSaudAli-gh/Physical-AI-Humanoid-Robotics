# Data Model for RAG Agent

## Entity: ChatRequest
- **Description**: Represents a user's request to the chat endpoint with query and optional selected_text
- **Fields**:
  - query: string (the main question from the user)
  - selected_text: string (optional text that the user has highlighted or selected, default: null)

## Entity: ChatResponse
- **Description**: Represents the agent's response with answer and array of source references
- **Fields**:
  - answer: string (the agent's response to the user's query)
  - sources: array of SourceReference (list of sources used to generate the answer)

## Entity: RetrievedContext
- **Description**: Represents the content chunks retrieved from the RAG system that inform the agent's response
- **Fields**:
  - content: string (the text content of the retrieved chunk)
  - source_url: string (URL of the original source)
  - page_title: string (title of the original page)
  - similarity_score: float (similarity score between query and content)
  - chunk_metadata: object (additional metadata stored with the chunk)

## Entity: SourceReference
- **Description**: Represents a reference to the original content that contributed to the agent's answer
- **Fields**:
  - source_url: string (URL of the original source)
  - page_title: string (title of the original page)
  - snippet: string (relevant snippet from the source)
  - relevance_score: float (relevance score of this source to the query)

## Relationships
- One ChatRequest can result in one ChatResponse (1 to 1)
- One ChatResponse contains many SourceReferences (1 to many)
- One ChatRequest may use many RetrievedContext items to generate the response (1 to many)

## Validation Rules
- ChatRequest query must not be empty or contain only whitespace
- selected_text in ChatRequest is optional and can be null
- ChatResponse answer must be provided and not empty
- ChatResponse sources must be an array (can be empty if no sources found)
- SourceReference must include source_url and page_title
- RetrievedContext similarity_score must be between 0 and 1