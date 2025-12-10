# Physical AI & Humanoid Robotics Textbook

## Development Environment Setup

This project requires Python 3.11+ and Node.js 18+.

### Backend Setup (Python 3.11+)

1. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. Install dependencies:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

4. Run the backend:
   ```bash
   cd backend
   uvicorn src.main:app --reload
   ```

### Frontend Setup (Node.js 18+)

1. Install dependencies:
   ```bash
   cd frontend
   npm install
   ```

2. Run the development server:
   ```bash
   cd frontend
   npm start
   ```

### Environment Variables

Create a `.env` file in the backend directory with the following:

```env
DATABASE_URL="postgresql://user:password@localhost:5432/textbook_db"
QDRANT_URL="https://your-cluster-url.qdrant.tech:6333"
COHERE_API_KEY="your-cohere-api-key"
OPENAI_API_KEY="your-openai-api-key"
BETTER_AUTH_SECRET="your-auth-secret"
BETTER_AUTH_URL="http://localhost:3000"
```

## Project Structure

- `backend/` - FastAPI application for backend services
- `frontend/` - Docusaurus application for the textbook UI
- `.qwen/` - Qwen CLI subagents and skills for content generation