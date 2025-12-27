# Quickstart Guide for RAG Agent

## Prerequisites

- Python 3.11+
- Access to OpenAI API (with appropriate API key)
- Completed retrieval service from Spec 2 (retrieval_service.py must be accessible)
- UV package manager

## Setup

1. Navigate to the rag-backend directory:
   ```bash
   cd rag-backend
   ```

2. Install dependencies:
   ```bash
   uv pip install fastapi uvicorn openai python-dotenv pydantic httpx pytest
   ```

3. Ensure your `.env` file contains the required API keys:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   ```

## Running the Agent Service

1. To start the FastAPI server:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

2. The API will be available at `http://localhost:8000`
3. API documentation will be available at `http://localhost:8000/docs`

## Using the API

1. To send a question to the agent using curl:
   ```bash
   curl -X POST "http://localhost:8000/api/chat" \
   -H "Content-Type: application/json" \
   -d '{
     "query": "What is ROS 2?",
     "selected_text": "Robot Operating System"
   }'
   ```

2. Expected response format:
   ```json
   {
     "answer": "ROS 2 (Robot Operating System 2) is a set of libraries and tools...",
     "sources": [
       {
         "source_url": "https://syedsaudali-gh.github.io/Physical-AI-Humanoid-Robotics/ros2/introduction",
         "page_title": "Introduction to ROS 2",
         "snippet": "ROS 2 is a flexible framework for writing robot applications...",
         "relevance_score": 0.92
       }
     ]
   }
   ```

## Running Tests

1. To run unit tests:
   ```bash
   pytest tests/test_agent.py -v
   ```

2. To run API tests:
   ```bash
   pytest tests/test_api.py -v
   ```

3. To run integration tests:
   ```bash
   pytest tests/test_integration.py -v
   ```

4. To run all tests:
   ```bash
   pytest
   ```

## Configuration

The agent can be configured through environment variables in the `.env` file:
- OPENAI_API_KEY: Your OpenAI API key for agent functionality
- Additional configuration parameters can be added to config.py as needed

## Example Usage

```python
import httpx

async def ask_question(query, selected_text=None):
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "http://localhost:8000/api/chat",
            json={
                "query": query,
                "selected_text": selected_text
            }
        )
        return response.json()

# Example usage
result = await ask_question("Explain how Qdrant works?", "vector database")
print(result["answer"])
print("Sources:", result["sources"])
```

## Troubleshooting

- If the API returns 500 errors, check that the retrieval service from Spec 2 is running and accessible
- If OpenAI API calls fail, verify your API key is correct and has sufficient quota
- For CORS issues, ensure the server is configured to accept requests from your frontend domain