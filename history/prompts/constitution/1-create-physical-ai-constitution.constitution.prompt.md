---
id: 1
title: Create Physical AI Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: gpt-4o
feature: none
branch: main
user: qwen_user
command: /sp.constitution
labels: [constitution, physical-ai, textbook, robotics, education]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
  - No automated tests for constitution creation
---

## Prompt

```
/sp.constitution
Project: Create a Textbook for Teaching Physical AI & Humanoid Robotics Course
Core Principles:
- Accuracy through verification from primary or authoritative sources in AI, robotics, and related fields.
- Clarity suitable for students with a background in AI, computer science, and basic engineering, ensuring concepts are explained progressively.
- Comprehensiveness: Cover theoretical foundations, practical implementations, and hands-on exercises for embodied intelligence.
- Integration of Tools: Utilize Qwen CLI and Spec-Kit Plus for generating content, ensuring AI-driven creation with reusable intelligence via subagents and agent skills.
- Interactivity and Personalization: Incorporate features for user personalization, translation, and interactive elements like RAG chatbot.
Key Standards:
- All technical claims must be linked to verifiable sources such as official documentation (e.g., ROS 2, NVIDIA Isaac, Gazebo).
- Use consistent formatting with Docusaurus for the book structure, including markdown for chapters and deployment to GitHub Pages.
- At least 50% of content must include practical code examples, simulations, and exercises using tools like ROS 2, Gazebo, Unity, and NVIDIA Isaac.
- Plagiarism tolerance: 0% (all content must be originally generated or properly attributed).
- Writing clarity target: Flesch-Kincaid Grade Level 8–10 for accessibility to undergraduate/graduate students.
- Structure the book into 4 modules, each with 3-4 chapters, aligned with the weekly breakdown, including detailed explanations, learning outcomes, and assessments.
- Implement integrated RAG chatbot using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier, capable of answering questions on book content and user-selected text.
- Bonus Features: Include reusable intelligence via Qwen CLI Subagents and Agent Skills; implement signup/signin with Better-Auth.com (asking about user's software/hardware background); enable logged-in users to personalize chapter content and translate to Urdu via buttons at chapter starts.
Constraints:
- Book structure: 4 modules with chapters (e.g., Module 1: 4 chapters on ROS 2; Module 2: 3 chapters on Gazebo & Unity; Module 3: 4 chapters on NVIDIA Isaac; Module 4: 3 chapters on VLA), plus introductory/overview sections, assessments, and hardware requirements.
- Total chapter count: 12-16 chapters, distributed across modules, with detailed content on topics like ROS 2 nodes/topics/services, URDF, Gazebo simulations, NVIDIA Isaac perception/navigation, VLA with voice-to-action and cognitive planning.
- Include capstone project description and hardware architecture details (e.g., Digital Twin Workstation, Edge Kits, Robot Options).
- Final output: Deployed Docusaurus site on GitHub Pages with embedded RAG chatbot and bonus features for up to 200 extra points.
Success Criteria:
- Every technical concept is validated against cited sources and includes practical examples.
- Zero plagiarism detected in the generated content.
- Book successfully deploys with functional RAG chatbot, personalization, and translation features.
- Achieves bonus points by implementing subagents/skills, auth system, personalization, and Urdu translation.
```

## Response snapshot

The constitution has been successfully updated with the following:
- **New Version**: 1.0.0 (initial version for this project)
- **Version Bump Rationale**: This is the first version of the constitution for the Physical AI & Humanoid Robotics Course Textbook project, establishing the foundational principles and standards. Since this is a new constitution rather than an update to an existing one, it begins at version 1.0.0.
- **Files Updated**: 
  - .specify/memory/constitution.md (main constitution file)
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
- **Suggested Commit Message**: `docs: create constitution for Physical AI & Humanoid Robotics textbook project`

The constitution now includes six core principles based on your requirements:
1. Accuracy through Verification
2. Clarity and Progressive Learning
3. Comprehensive Coverage
4. Tool Integration
5. Interactivity and Personalization
6. Zero Plagiarism Standard

It also includes the Key Standards, Constraints, Success Criteria, and Governance sections as specified in your request. The dependent templates have been updated to align with these new principles and standards.

## Outcome

- ✅ Impact: Created new project constitution with 6 core principles and supporting sections
- 🧪 Tests: No automated tests for constitution creation
- 📁 Files: 4 files updated to align with new constitution
- 🔁 Next prompts: Ready for next phase of textbook development
- 🧠 Reflection: Successfully implemented user's requirements with appropriate template adjustments

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A