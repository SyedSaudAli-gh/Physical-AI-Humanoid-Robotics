# Physical AI & Humanoid Robotics Textbook

This project implements a comprehensive textbook for Physical AI & Humanoid Robotics with interactive features including authentication, RAG chatbot, personalization, and Urdu translation.

## Features Implemented

1. **Authentication System**
   - User signup with background questionnaire
   - Technical skills collection during registration
   - Secure login/logout functionality

2. **RAG (Retrieval Augmented Generation) Chatbot**
   - Integrates with Cohere for embeddings
   - Uses Qdrant for vector storage
   - Queries textbook content and selected text
   - Powered by OpenAI for responses

3. **Content Personalization**
   - Adapts chapter content based on user's experience level
   - Adjusts complexity based on technical skills
   - Tiered content adaptation (beginner, intermediate, advanced)

4. **Urdu Translation**
   - Translate chapter content to Urdu
   - Preserves technical terms during translation
   - Right-to-left text support for Urdu

5. **Qwen Subagents**
   - Content generation agent
   - Code assistance agent
   - Translation agent
   - Simulation configuration agent
   - Textbook generator agent

## Setup Instructions

### Backend Setup

1. Navigate to the rag-backend directory:
```bash
cd rag-backend
```

2. Install dependencies using uv (as specified in the project):
```bash
uv pip install -e .
```

3. Set up environment variables in `.env`:
```env
# Backend Environment Variables
OPENAI_API_KEY=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
COHERE_API_KEY=HTFZezITzJtoLBloDX0rP4Eb6NKsrk9BTxNkNW7l
QDRANT_URL=https://c96efe7c-aa83-47e9-a297-2961f5942f0c.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.3CuOHUNlKyj01GPjTuQavvfNsYy0n2gjdS3IWfcM7q0
DATABASE_URL=https://ep-wild-bird-adcyfk2v.apirest.c-2.us-east-1.aws.neon.tech/neondb/rest/v1
BETTER_AUTH_SECRET=MTPMFZy6ovucemA62babULjzW07s6DV9
GEMINI_API_KEY=AIzaSyCURhNq2jgupaiJs1yS_oatEMTy9LaJcbY
BETTER_AUTH_URL="http://localhost:3000"
```

4. Start the rag-backend server:
```bash
cd rag-backend
uv run main.py
```

### Frontend Setup

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

## Running the Upload Script

To upload chapters to the vector database:

```bash
# The upload script functionality is now part of the rag-backend pipeline
cd rag-backend
python -c "from main import run_ingestion_pipeline; run_ingestion_pipeline()"
```

## Testing the Implementation

Run the comprehensive test to validate all features:

```bash
cd rag-backend
# Run the specific tests for rag-backend functionality
python -m pytest tests/
```

## Project Structure

```
Physical-AI-Humanoid-Robotics/
├── rag-backend/
│   ├── agent.py          # AI agent implementation using OpenAI Agents SDK
│   ├── main.py           # FastAPI application entry point
│   ├── models.py         # Pydantic models for request/response validation
│   ├── tools.py          # Tools for the agent (retrieval service integration)
│   ├── config.py         # Configuration settings
│   ├── retrieval_service.py # RAG retrieval functionality
│   ├── pyproject.toml    # Project dependencies and configuration
│   └── tests/            # Backend tests
├── frontend/
│   ├── docs/             # Textbook content
│   ├── src/
│   │   ├── components/   # React components
│   │   ├── contexts/     # React contexts (AuthContext)
│   │   ├── pages/        # Docusaurus pages
│   │   └── Root.js       # Global component injection
│   └── docusaurus.config.js
└── .qwen/
    ├── subagents/        # Qwen CLI subagents
    └── skills/           # Qwen CLI skills
```

## API Endpoints

- `POST /api/auth/signup` - User registration with questionnaire
- `POST /api/auth/login` - User login
- `POST /api/personalize` - Get personalized chapter content
- `POST /api/translate/chapter-urdu` - Translate chapter to Urdu
- `POST /api/chat/query` - RAG chatbot query
- `POST /api/chat/process-chapter` - Process chapter for RAG system

## Qwen Subagents Usage

The project includes 5 Qwen subagents:

1. `textbook_generator.py` - Generates textbook content
2. `translator.py` - Handles translation tasks
3. `code_assistant.py` - Generates robotics code examples
4. `content_generator.py` - Creates educational content
5. `simulation_config.py` - Configures simulation environments

Run any subagent via:
```bash
cd .qwen/subagents
python -m code_assistant --task "create a ROS navigation node"
```

## Environment Configuration

Make sure to properly set up your `.env` file with all the required API keys and database URLs. The application includes graceful degradation when external services are unavailable.

### How Environment Variables Are Used

**Backend (Python/FastAPI):**
- The backend uses `python-dotenv` library to load environment variables from the `.env` file
- Configuration is handled through the `Settings` class in `backend/src/config.py`
- API keys are accessed via `os.getenv()` and integrated into services like OpenAI, Cohere, and Qdrant
- The main application entry point (`backend/src/main.py`) calls `load_dotenv()` to ensure environment variables are loaded

**Frontend (Docusaurus/React):**
- For Docusaurus applications, environment variables are processed at build time
- The frontend is configured to proxy API requests to the backend via the development server
- All external API calls (OpenAI, Cohere, etc.) should be made through backend endpoints for security
- Configuration is handled in `frontend/src/services/config.js`

### Security Best Practices

- Keep your `.env` file in `.gitignore` to prevent committing sensitive information
- Never expose API keys directly in frontend code
- All external API calls should be proxied through the backend
- Use different API keys for development and production environments
- Regularly rotate API keys and review usage limits

## Deployment

For production deployment:

1. Set up a production database
2. Configure environment variables
3. Build the frontend: `npm run build`
4. Deploy to your preferred hosting platform

For GitHub Pages deployment of the frontend, follow Docusaurus deployment instructions specific to your repository.