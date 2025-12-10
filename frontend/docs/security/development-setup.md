# Setting Up Your Development Environment

This guide will help you set up your local development environment for the Physical AI & Humanoid Robotics project.

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git version control system

## Initial Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Create your local environment file:
   ```bash
   cp .env.example .env
   ```

## Environment Configuration

This project follows security best practices for handling environment variables:

1. **Backend Variables** (kept separately in your backend directory):
   - API keys for external services (OpenAI, Cohere, Google Gemini, etc.)
   - Database connection strings
   - Authentication secrets

2. **Frontend Variables** (in the .env file):
   - API base URLs
   - Analytics IDs (if applicable)

> ⚠️ Important: Never commit actual API keys to version control. Always use .env files that are ignored by git.

## Running the Development Server

```bash
npm start
```

This will start the Docusaurus development server, typically available at http://localhost:3000

## Security Guidelines

- Store all sensitive API keys in the backend
- Never expose backend API keys to frontend code
- Use API proxy pattern for external service calls
- Follow the environment variable security guide in the documentation

## Building for Production

```bash
npm run build
```

This command will build the static site for production deployment.