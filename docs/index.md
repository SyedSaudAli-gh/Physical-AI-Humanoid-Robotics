# Physical AI & Humanoid Robotics Textbook Documentation

## Overview

This project implements a comprehensive textbook for Physical AI & Humanoid Robotics, designed to bridge digital minds to physical bodies. The textbook covers 4 core modules:

1. **ROS 2** - The Robotic Nervous System
2. **Gazebo & Unity** - The Digital Twin 
3. **NVIDIA Isaac** - AI-Driven Perception
4. **Vision-Language-Action (VLA)** - Conversational Robotics

## Architecture

### Backend (FastAPI)
- **Models**: User, Module, Chapter, BookContent, RagIndex, InteractionLog, TranslationData
- **Services**: ModuleService, ChapterService, RAGService, TranslationService, ValidationService
- **APIs**: Modules, Chapters, Authentication, Chat, Translation
- **Database**: PostgreSQL for user data and content management
- **Vector DB**: Qdrant Cloud for RAG functionality

### Frontend (Docusaurus)
- **Components**: RAGChatbot, ChapterContent, UserRegistration, LanguageSwitcher
- **Authentication**: Integrated with Better-Auth
- **Internationalization**: Support for English and Urdu
- **Content Personalization**: Based on user skills and preferences

## Features

### Educational Content Access (User Story 1)
- 4 core modules with 3-4 detailed chapters each
- Theoretical foundations, practical simulations, and code examples
- Interactive navigation between modules and chapters

### RAG Chatbot (User Story 2)
- Integration with Cohere embeddings for semantic search
- Vector storage in Qdrant Cloud
- Contextual answers based on textbook content
- Text selection and context passing

### User Registration & Personalization (User Story 3)
- Secure signup with background questionnaire
- Collection of technical skills and experience level
- Content personalization based on user profile
- Difficulty level adjustments

### Urdu Translation (User Story 4)
- Qwen-powered translation service
- UI controls for language switching
- Right-to-left layout support for Urdu
- Translation verification workflow

## Development Setup

### Backend
```bash
cd backend
pip install -r requirements.txt
uvicorn src.main:app --reload
```

### Frontend
```bash
cd frontend
npm install
npm start
```

## API Endpoints

### Modules
- `GET /api/modules` - Get all textbook modules
- `GET /api/modules/{module_id}/chapters` - Get chapters in a module

### Chapters
- `GET /api/chapters/{chapter_id}` - Get specific chapter content

### Authentication
- `POST /api/auth/signup` - User registration
- `POST /api/auth/login` - User login

### Chat
- `POST /api/chat/query` - Query RAG system

### Translation
- `POST /api/translate` - Translate content

## Configuration

### Environment Variables
Create `.env` file in backend directory:
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

### Deployment
- Frontend: Deploy to GitHub Pages
- Backend: Deploy to Render.com or similar platform

## Qwen CLI Integration

The system includes AI-powered content generation tools:

### Content Generation
```bash
qwen-cli subagent textbook generate-content --module "ROS 2" --title "Understanding Nodes"
```

### Validation Tools
```bash
qwen-cli skill content-validator validate --source-type "ros2" --content-file "chapter.txt"
qwen-cli skill readability check --content-file "chapter.txt"
```

## Quality Standards

- Accuracy through verification with authoritative sources
- Clarity suitable for students with AI/computer science background (Flesch-Kincaid ≤ 10)
- Comprehensive coverage of theoretical and practical implementations
- Zero plagiarism in all generated content
- Reusable intelligence through Qwen CLI Subagents and Agent Skills

## Performance Targets

- Page load < 5s for 90% of users
- RAG response time < 3s
- Support for 100-500 concurrent users
- 95% uptime for core functionality

## Security & Privacy

- Secure authentication via Better-Auth
- Proper data protection for user background information and interaction logs
- API keys managed securely
- User privacy maintained during RAG processing