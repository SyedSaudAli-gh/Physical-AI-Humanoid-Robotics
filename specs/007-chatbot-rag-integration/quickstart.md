# Quickstart Guide: Docusaurus Chatbot ↔ RAG Backend Integration

## Prerequisites
- Node.js >= 18.0
- Python >= 3.9
- npm or yarn package manager
- Git

## Setup Instructions

### 1. Clone the Repository
```bash
git clone [repository-url]
cd [repository-directory]
```

### 2. Set up the Backend (RAG Service)
```bash
# Navigate to the backend directory
cd rag-backend

# Create a virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
# Or if using poetry: poetry install
# Or if using pyproject.toml: pip install -e .

# Set up environment variables (if needed)
cp .env.example .env
# Edit .env with your API keys and configuration

# Start the backend service
uvicorn main:app --reload --host 127.0.0.1 --port 8000
```

The backend should now be running at `http://127.0.0.1:8000` with the chat endpoint available at `http://127.0.0.1:8000/api/chat`.

### 3. Set up the Frontend (Docusaurus)
```bash
# Open a new terminal and navigate to the frontend directory
cd frontend

# Install dependencies
npm install
# Or if using yarn: yarn install

# Start the development server
npm start
# Or if using yarn: yarn start
```

The frontend should now be running at `http://localhost:3000`.

## Usage

1. Ensure both the backend and frontend services are running
2. Navigate to `http://localhost:3000` in your browser
3. Use the floating chatbot button to open the chat interface
4. Type your question in the input field and press Enter or click Send
5. The chatbot will send your query to the backend and display the AI-generated response

## Configuration

### Backend Configuration
- The backend service configuration can be modified in `rag-backend/config.py`
- Default host and port are set to `127.0.0.1:8000`

### Frontend Configuration
- API endpoints are defined in `frontend/src/config/constants.js`
- The chat endpoint is configured as `API_ENDPOINTS.CHAT = '/api/chat'`

## Troubleshooting

### Common Issues

1. **Backend not responding**: Ensure the backend service is running at `http://127.0.0.1:8000`
2. **CORS errors**: The backend should have CORS configured to allow requests from `http://localhost:3000`
3. **Timeout errors**: Check if the backend is taking too long to respond to complex queries

### Testing the API Connection
You can test the backend API directly using curl:
```bash
curl -X POST "http://127.0.0.1:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{"query": "Hello, are you working?"}'
```

## Development

### Running Tests
Backend tests:
```bash
cd rag-backend
python -m pytest tests/
```

Frontend tests:
```bash
cd frontend
npm test
# Or: yarn test
```

### API Documentation
The backend provides interactive API documentation at `http://127.0.0.1:8000/docs` when running in development mode.