# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Project Setup

### Prerequisites
- Node.js 18+ 
- Python 3.11+
- Git
- Docker (optional, for testing code examples)

### Initial Setup

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install frontend dependencies:**
   ```bash
   cd frontend
   npm install
   ```

3. **Install backend dependencies:**
   ```bash
   cd ../backend
   pip install -r requirements.txt
   ```

4. **Set up environment variables:**
   Create `.env` file in backend directory:
   ```env
   DATABASE_URL="your_neon_postgres_connection_string"
   QDRANT_URL="your_qdrant_cloud_url"
   COHERE_API_KEY="your_cohere_api_key"
   OPENAI_API_KEY="your_openai_api_key"
   BETTER_AUTH_SECRET="your_auth_secret"
   BETTER_AUTH_URL="http://localhost:3000"  # or your deployed URL
   ```

## Running the Application

### For Development

1. **Start the backend:**
   ```bash
   cd backend
   python -m src.main  # or however your FastAPI app is run
   ```

2. **Start the Docusaurus frontend:**
   ```bash
   cd frontend
   npm start
   ```

### For Production Build

1. **Build the Docusaurus site:**
   ```bash
   cd frontend
   npm run build
   ```

2. **Deploy to GitHub Pages:**
   The build output in `frontend/build` can be deployed to GitHub Pages

## Content Creation with Qwen CLI

### Using Subagents

1. **Generate a new chapter:**
   ```bash
   qwen-cli subagent textbook create-chapter --module "ROS 2" --title "Understanding ROS 2 Nodes" --output docs/ros2/understanding-nodes.md
   ```

2. **Translate content to Urdu:**
   ```bash
   qwen-cli subagent translator translate --input docs/ros2/example.md --target-language ur --output docs/ros2/example.ur.md
   ```

3. **Generate code examples:**
   ```bash
   qwen-cli skill generate-code-example --topic "ROS 2 Publisher" --language python --output examples/ros2-publisher.py
   ```

### Content Validation

1. **Check content against official documentation:**
   ```bash
   qwen-cli skill validate-content --input docs/ros2/understanding-nodes.md --source "ROS 2 Official Docs"
   ```

2. **Verify readability (Flesch-Kincaid ≤ 10):**
   ```bash
   qwen-cli skill check-readability --input docs/ros2/understanding-nodes.md
   ```

## Working with the RAG System

### Indexing New Content
1. Content is automatically indexed when added to the system
2. Embeddings are generated using Cohere's embed-multilingual-v3.0 model
3. Chunks are created at 512-token intervals with 128-token overlap

### Querying the RAG System
1. Send queries to `/api/chat/query` endpoint
2. Optionally include selected text for more targeted responses
3. Responses include source citations for transparency

## Managing Personalization

### For Developers
1. User background information is collected during signup
2. Content is served based on technical skills and experience level
3. Personalization can be overridden in user preferences

### For Content Creators
1. Create content variants for different difficulty levels
2. Tag technical concepts with appropriate difficulty labels
3. Test content at different levels to ensure appropriateness

## Deployment

### Frontend to GitHub Pages
1. The Docusaurus build process creates a static site
2. Use GitHub Actions workflow in `.github/workflows/deploy.yml` for automatic deployment

### Backend to Render.com
1. Connect your GitHub repository to Render.com
2. Configure environment variables in Render dashboard
3. Set up automatic deployments from the main branch

## Testing

### Unit Tests
```bash
# Backend tests
cd backend
pytest

# Frontend tests
cd frontend
npm test
```

### Integration Tests
1. Test RAG functionality with predefined question sets
2. Verify personalization works as expected
3. Test Urdu translation functionality

### Performance Testing
1. Load test with 100-500 concurrent users
2. Verify RAG response times under load
3. Test GitHub Pages performance metrics