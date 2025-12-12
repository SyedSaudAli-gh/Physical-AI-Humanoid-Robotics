import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

from .api import modules, chapters, auth, users, translation, personalization, chat
from .database import create_tables
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Initialize database tables
    create_tables()
    yield

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for the Physical AI & Humanoid Robotics Textbook",
    version="0.1.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins during development; restrict in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],  # Allow all headers initially
    # Expose headers for frontend access
    expose_headers=["Access-Control-Allow-Origin", "Authorization", "X-Total-Count"]
)

# Include API routers
app.include_router(modules.router, prefix="/api")
app.include_router(chapters.router, prefix="/api")
app.include_router(auth.auth_router, prefix="/api")
app.include_router(users.router, prefix="/api")
app.include_router(translation.router, prefix="/api")
app.include_router(personalization.router, prefix="/api")

@app.get("/")
def read_root():
    return {"message": "Welcome to the Physical AI & Humanoid Robotics Textbook API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)