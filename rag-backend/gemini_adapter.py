"""
Gemini API adapter to work with OpenAI Agent SDK as an orchestrator
"""
import logging
import google.generativeai as genai
from typing import Dict, List, Any, Optional
from config import GEMINI_API_KEY, GEMINI_MODEL
from tools import retrieve_content_tool

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configure Gemini API
genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel(GEMINI_MODEL)

# Define the tools available to the agent
def get_relevant_content(query: str, top_k: int = 5) -> Dict[str, Any]:
    """
    Tool to retrieve relevant content from the knowledge base
    """
    try:
        results = retrieve_content_tool(query, top_k)
        return {
            "status": "success",
            "results": results
        }
    except Exception as e:
        logger.error(f"Error retrieving content: {e}")
        return {
            "status": "error",
            "message": str(e)
        }

# Available tools for the agent
available_tools = {
    "get_relevant_content": get_relevant_content
}

def call_gemini_with_tools(messages: List[Dict[str, str]], tools: Optional[List[Dict]] = None) -> str:
    """
    Call Gemini with optional tools
    """
    try:
        # Convert messages to Gemini format
        gemini_contents = []
        for msg in messages:
            role = "model" if msg["role"] == "assistant" else "user"
            gemini_contents.append({
                "role": role,
                "parts": [{"text": msg["content"]}]
            })
        
        # Prepare tools if provided
        if tools:
            # For now, we'll just include the tool information in the system context
            # In a more advanced implementation, we would handle tool calling properly
            system_instruction = f"""
            You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
            You have access to tools to retrieve relevant content from the textbook.
            The available tools are: {list(available_tools.keys())}
            
            When appropriate, you can call these tools to get more information.
            """
            
            response = model.generate_content(
                contents=gemini_contents,
                system_instruction=system_instruction
            )
        else:
            response = model.generate_content(gemini_contents)
        
        return response.text
        
    except Exception as e:
        logger.error(f"Error calling Gemini: {e}")
        raise e