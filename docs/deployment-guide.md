# Deployment Instructions for Physical AI & Humanoid Robotics Textbook

## Frontend (Docusaurus) Deployment to GitHub Pages

### Prerequisites
- GitHub account
- Repository set up for the project
- Admin rights to configure GitHub Pages

### Steps
1. Commit all changes to your repository:
   ```bash
   git add .
   git commit -m "Prepare for GitHub Pages deployment"
   git push origin main
   ```

2. Navigate to your repository on GitHub and go to Settings > Pages

3. Under "Source", select "Deploy from a branch"

4. Select "main" branch and "/root" folder, then click "Save"

5. In your project's root directory, ensure the `homepage` in `frontend/package.json` is set correctly:
   ```json
   {
     "name": "physical-ai-textbook",
     "version": "0.0.0",
     "private": true,
     "scripts": {
       "docusaurus": "docusaurus",
       "start": "docusaurus start",
       "build": "docusaurus build",
       "swizzle": "docusaurus swizzle",
       "deploy": "docusaurus deploy",
       "clear": "docusaurus clear",
       "serve": "docusaurus serve",
       "write-translations": "docusaurus write-translations",
       "write-heading-ids": "docusaurus write-heading-ids"
     },
     "browserslist": {
       "production": [
         ">0.5%",
         "not dead",
         "not op_mini all"
       ],
       "development": [
         "last 1 chrome version",
         "last 1 firefox version",
         "last 1 safari version"
       ]
     },
     "homepage": "https://your-username.github.io/your-repo-name"
   }
   ```

6. Build the frontend:
   ```bash
   cd frontend
   npm run build
   ```

7. The GitHub Actions workflow (`.github/workflows/deploy.yml`) will automatically deploy the frontend to GitHub Pages after merging to main.

## Backend (FastAPI) Deployment to Render

### Prerequisites
- Render.com account
- Repository connected to Render

### Steps
1. Create a new Web Service on Render
   - Connect to your GitHub repository
   - Choose the root directory
   - Environment: Python
   - Branch: main

2. Add build command:
   ```bash
   pip install -r backend/requirements.txt
   ```

3. Add start command:
   ```bash
   cd backend && uvicorn src.main:app --host 0.0.0.0 --port $PORT
   ```

4. Configure environment variables in Render dashboard:
   - DATABASE_URL (PostgreSQL add-on)
   - QDRANT_URL
   - COHERE_API_KEY
   - BETTER_AUTH_SECRET
   - BETTER_AUTH_URL (should be your Render service URL)

5. Render will automatically deploy when you push to the main branch.

## GitHub Actions Configuration

A GitHub Actions workflow is already configured in `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest
    
    steps:
    - name: Checkout 🛎️
      uses: actions/checkout@v3

    - name: Setup Node.js 🧰
      uses: actions/setup-node@v3
      with:
        node-version: '18'
        cache: 'npm'
        cache-dependency-path: frontend/package-lock.json

    - name: Install dependencies 📦
      run: |
        cd frontend
        npm ci

    - name: Build 🏗️
      run: |
        cd frontend
        npm run build
      env:
        NODE_OPTIONS: '--max_old_space_size=4096'

    - name: Deploy to GitHub Pages 🚀
      uses: peaceiris/actions-gh-pages@v3
      with:
        github_token: ${{ secrets.GITHUB_TOKEN }}
        publish_dir: ./frontend/build
        # Enable this if you want to deploy to a custom domain
        # cname: your-custom-domain.com
```

## Verification Steps

After deployment:

1. Visit your GitHub Pages URL: `https://your-username.github.io/your-repo-name`

2. Verify the backend is running by accessing: `https://your-service.onrender.com/health`

3. Test the textbook functionality:
   - Browse modules and chapters
   - Test the RAG chatbot
   - Test user registration
   - Test Urdu translation functionality

4. Check that all integrations work (Cohere, Qdrant, Better Auth)

## Post-Deployment Checklist

- [ ] Frontend is accessible on GitHub Pages
- [ ] Backend API is responding on Render
- [ ] RAG chatbot is functioning
- [ ] User registration and authentication work
- [ ] Content personalization based on user profile works
- [ ] Urdu translation functionality works
- [ ] All external services (Cohere, Qdrant) are properly integrated
- [ ] Security measures are active
- [ ] Monitoring and logging are operational

## Troubleshooting

- If GitHub Pages doesn't load, ensure the `homepage` in frontend/package.json is correct
- If backend endpoints return 404, check the API gateway settings on Render
- If authentication fails, verify that BETTER_AUTH_URL points to your deployed backend
- Check environment variables in both GitHub and Render dashboards
- Monitor the logs in the Render dashboard for backend issues
- Check browser console for frontend errors