"""
RAG Agent implementation for the RAG Chatbot application.

This agent acts as an orchestrator:
- Retrieves relevant context from Qdrant via tools
- Uses Gemini 2.5 Flash as the LLM
- Returns structured responses for FastAPI
"""

import logging
from typing import List

from tools import get_relevant_context
from models import ChatRequest, ChatResponse, SourceReference
from gemini_adapter import call_gemini_with_tools

# --------------------------------------------------
# Logging Configuration
# --------------------------------------------------
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RAGAgent:
    """
    RAG Agent that orchestrates retrieval + generation.
    Gemini is used as the LLM (no OpenAI client required).
    """

    def __init__(self):
        logger.info("RAGAgent initialized (Gemini LLM)")

    def answer_question(self, chat_request: ChatRequest) -> ChatResponse:
        """
        Answer a user question using retrieved context and Gemini.

        Args:
            chat_request (ChatRequest): User query and optional selected_text

        Returns:
            ChatResponse: Answer with optional source references
        """
        try:
            logger.info(f"Received query: {chat_request.query[:80]}")

            # --------------------------------------------------
            # 1. Retrieve Context
            # --------------------------------------------------
            context = get_relevant_context(
                query=chat_request.query,
                selected_text=chat_request.selected_text
            )

            if not context or not context.strip():
                logger.warning("No context found, falling back to general knowledge")
                context = (
                    "No relevant content was found in the textbook. "
                    "Answer using general knowledge and clearly state this limitation."
                )

            # --------------------------------------------------
            # 2. Build Prompt
            # --------------------------------------------------
            prompt = self._build_prompt(
                query=chat_request.query,
                context=context
            )

            messages = [
                {
                    "role": "system",
                    "content": (
                        "You are an AI assistant helping students with the "
                        "Physical AI & Humanoid Robotics textbook. "
                        "Answer ONLY using the provided context when possible. "
                        "If the context is insufficient, clearly say so."
                    )
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ]

            # --------------------------------------------------
            # 3. Generate Answer (Gemini)
            # --------------------------------------------------
            answer_text = call_gemini_with_tools(messages)

            # --------------------------------------------------
            # 4. Extract Sources
            # --------------------------------------------------
            sources = self._extract_sources_from_context(context)

            logger.info(f"Answer generated successfully | Sources: {len(sources)}")

            return ChatResponse(
                answer=answer_text,
                sources=sources
            )

        except Exception as exc:
            logger.exception("Error while processing chat request")
            raise exc

    # --------------------------------------------------
    # Helper Methods
    # --------------------------------------------------
    def _build_prompt(self, query: str, context: str) -> str:
        """
        Build the final prompt sent to Gemini.
        """
        return f"""
Context:
{context}

Question:
{query}

Instructions:
- Answer clearly and concisely.
- Use ONLY the context above if possible.
- If the answer is not present, say so explicitly.
""".strip()

    def _extract_sources_from_context(self, context: str) -> List[SourceReference]:
        """
        Extract structured source references from the context string.
        Expected separator: '---'
        """
        sources: List[SourceReference] = []

        parts = context.split("---")

        for part in parts:
            try:
                if "Source:" not in part:
                    continue

                def _extract(label: str) -> str:
                    start = part.find(label) + len(label)
                    end = part.find("\n", start)
                    return part[start:end].strip()

                source_url = _extract("Source:")
                page_title = _extract("Title:")
                snippet = _extract("Content:")

                if "Relevance Score:" in part:
                    score_start = part.find("Relevance Score:") + len("Relevance Score:")
                    score_end = part.find("\n", score_start)
                    relevance_score = float(part[score_start:score_end].strip())
                else:
                    relevance_score = 0.5

                sources.append(
                    SourceReference(
                        source_url=source_url,
                        page_title=page_title,
                        snippet=snippet,
                        relevance_score=relevance_score
                    )
                )

            except Exception as e:
                logger.warning(f"Failed to parse source block: {e}")

        return sources


# --------------------------------------------------
# Global Agent Instance
# --------------------------------------------------
rag_agent = RAGAgent()


def process_chat_request(chat_request: ChatRequest) -> ChatResponse:
    """
    Entry point used by FastAPI.
    """
    return rag_agent.answer_question(chat_request)
