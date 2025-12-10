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

1. Navigate to the backend directory:
```bash
cd backend
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up environment variables in `.env`:
```env
DATABASE_URL="postgresql://user:password@localhost:5432/textbook_db"
QDRANT_URL="https://your-cluster-url.qdrant.tech:6333"
COHERE_API_KEY="your-cohere-api-key"
OPENAI_API_KEY="your-openai-api-key"
BETTER_AUTH_SECRET="your-auth-secret"
BETTER_AUTH_URL="http://localhost:3000"
```

4. Start the backend server:
```bash
cd src
uvicorn main:app --reload
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
cd backend/src
python upload_chapters.py --docs-dir ../frontend/docs --load-docs
```

## Testing the Implementation

Run the comprehensive test to validate all features:

```bash
cd backend/src
python test_features.py
```

## Project Structure

```
Physical-AI-Humanoid-Robotics/
├── backend/
│   ├── src/
│   │   ├── api/          # API endpoints
│   │   ├── auth/         # Authentication logic
│   │   ├── models/       # Database models
│   │   ├── services/     # Business logic
│   │   ├── rag/          # RAG system components
│   │   └── main.py       # Main application entry point
│   └── requirements.txt
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

## Deployment

For production deployment:

1. Set up a production database
2. Configure environment variables
3. Build the frontend: `npm run build`
4. Deploy to your preferred hosting platform

For GitHub Pages deployment of the frontend, follow Docusaurus deployment instructions specific to your repository.