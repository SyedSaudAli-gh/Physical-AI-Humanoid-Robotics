# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-10 | **Spec**: [link]
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Develop a comprehensive textbook for Physical AI & Humanoid Robotics with 4 modules (ROS 2, Gazebo & Unity, NVIDIA Isaac, VLA) containing 15 chapters + 5 supporting sections. Implement with Docusaurus for content delivery, integrated RAG chatbot with Cohere embeddings, user personalization based on technical background, and Urdu translation functionality. Deploy to GitHub Pages with backend services using FastAPI, Neon Postgres, Qdrant Cloud, and Better-Auth for authentication.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript, Node.js 18+
**Primary Dependencies**: Docusaurus, FastAPI, Better-Auth, Cohere, OpenAI/ChatKit SDKs, Qdrant, Neon Postgres
**Storage**: GitHub Pages (frontend), Neon Serverless Postgres (user data), Qdrant Cloud (vector embeddings)
**Testing**: pytest, integration tests for RAG functionality, user acceptance tests
**Target Platform**: Web application (GitHub Pages + FastAPI backend)
**Project Type**: Web application with embedded interactive components
**Performance Goals**: Page load < 5s for 90% of users, RAG response time < 3s, support 100-500 concurrent users
**Constraints**: Must use Qwen CLI and Spec-Kit Plus for AI-driven content generation, content from authoritative sources (ROS 2, NVIDIA Isaac, Gazebo docs), Flesch-Kincaid ≤ 10 readability
**Scale/Scope**: 15 chapters (~2000-4000 words each), 4 modules, 100-500 concurrent users, multiple interactive features (RAG, personalization, translation)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

This project must comply with the Physical AI & Humanoid Robotics Course Textbook Constitution, ensuring:
- Accuracy through verification from primary or authoritative sources: ✅ (using official docs from ROS 2, NVIDIA Isaac, Gazebo, etc.)
- Clarity suitable for students with AI/computer science background: ✅ (Flesch-Kincaid ≤ 10 target)
- Comprehensive coverage of theoretical and practical implementations: ✅ (4 modules, 15 chapters with code examples)
- Integration of Qwen CLI and Spec-Kit Plus tools: ✅ (for content generation with subagents)
- Implementation of interactivity and personalization features: ✅ (RAG chatbot, personalization, Urdu translation)
- Zero plagiarism in all generated content: ✅ (original content with proper attribution)

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   ├── auth/
│   └── rag/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
├── docs/                # Docusaurus content (modules, chapters)
├── static/              # Static assets
└── tests/

api/
└── openapi.yaml         # API contract

# For Qwen CLI Subagents and Agent Skills
.qwen/
├── subagents/
└── skills/

# Deployment configuration
.github/
└── workflows/
    └── deploy.yml       # GitHub Pages deployment
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (Docusaurus) to handle complex interactive features like RAG chatbot, authentication, and personalization while maintaining static content delivery via GitHub Pages.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Separate backend | RAG functionality & auth require server-side processing | Client-only would expose API keys and limit functionality |