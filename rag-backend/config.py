"""
Configuration for the RAG Agent application.
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration settings
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")  # For OpenAI Agent SDK only
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY environment variable is required")

# API settings
API_HOST = os.getenv("API_HOST", "0.0.0.0")
API_PORT = int(os.getenv("API_PORT", "8000"))
API_RELOAD = os.getenv("API_RELOAD", "True").lower() == "true"

# Agent settings
AGENT_NAME = os.getenv("AGENT_NAME", "RAG-Agent")

# Retrieval settings
DEFAULT_TOP_K = int(os.getenv("DEFAULT_TOP_K", "5"))
RESPONSE_TIMEOUT_MS = int(os.getenv("RESPONSE_TIMEOUT_MS", "30000"))
MIN_SCORE_THRESHOLD = float(os.getenv("MIN_SCORE_THRESHOLD", "0.5"))
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "physical_ai_docs")

# Gemini settings
GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")