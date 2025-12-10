<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (New constitution)
Added sections: Core Principles (6), Key Standards, Constraints, Success Criteria
Removed sections: N/A
Templates requiring updates: 
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated  
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ✅ reviewed - no changes needed
Deferred items: None
-->

# Physical AI & Humanoid Robotics Course Textbook Constitution

## Core Principles

### Accuracy through Verification
All content must be validated against primary or authoritative sources in AI, robotics, and related fields. Technical claims MUST link to official documentation or peer-reviewed sources (e.g., ROS 2, NVIDIA Isaac, Gazebo).
<!-- Rationale: Ensures educational content maintains technical accuracy and credibility -->

### Clarity and Progressive Learning
Content must be accessible to students with backgrounds in AI, computer science, and basic engineering, with concepts explained progressively from foundational to advanced.
<!-- Rationale: Enables effective learning for the target audience with diverse skill levels -->

### Comprehensive Coverage
Each topic must include theoretical foundations, practical implementations, and hands-on exercises focused on embodied intelligence.
<!-- Rationale: Combines theory with practice to enhance understanding and application -->

### Tool Integration
Utilize Qwen CLI and Spec-Kit Plus for content generation, ensuring AI-driven creation with reusable intelligence via subagents and agent skills.
<!-- Rationale: Leverages modern AI tools to efficiently create and maintain high-quality content -->

### Interactivity and Personalization
Incorporate features for user personalization, translation, and interactive elements like RAG chatbot for enhanced learning experience.
<!-- Rationale: Provides adaptive and engaging learning experiences for diverse learners -->

### Zero Plagiarism Standard
All content must be originally generated or properly attributed with no tolerance for plagiarism.
<!-- Rationale: Maintains academic integrity and originality of the educational material -->


## Key Standards

- All technical claims must be linked to verifiable sources such as official documentation (e.g., ROS 2, NVIDIA Isaac, Gazebo).
- Use consistent formatting with Docusaurus for the book structure, including markdown for chapters and deployment to GitHub Pages.
- At least 50% of content must include practical code examples, simulations, and exercises using tools like ROS 2, Gazebo, Unity, and NVIDIA Isaac.
- Writing clarity target: Flesch-Kincaid Grade Level 8–10 for accessibility to undergraduate/graduate students.
- Structure the book into 4 modules, each with 3-4 chapters, aligned with the weekly breakdown, including detailed explanations, learning outcomes, and assessments.
- Implement integrated RAG chatbot using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier, capable of answering questions on book content and user-selected text.

## Constraints

- Book structure: 4 modules with chapters (e.g., Module 1: 4 chapters on ROS 2; Module 2: 3 chapters on Gazebo & Unity; Module 3: 4 chapters on NVIDIA Isaac; Module 4: 3 chapters on VLA), plus introductory/overview sections, assessments, and hardware requirements.
- Total chapter count: 12-16 chapters, distributed across modules, with detailed content on topics like ROS 2 nodes/topics/services, URDF, Gazebo simulations, NVIDIA Isaac perception/navigation, VLA with voice-to-action and cognitive planning.
- Include capstone project description and hardware architecture details (e.g., Digital Twin Workstation, Edge Kits, Robot Options).
- Final output: Deployed Docusaurus site on GitHub Pages with embedded RAG chatbot and bonus features for up to 200 extra points.

## Success Criteria

- Every technical concept is validated against cited sources and includes practical examples.
- Zero plagiarism detected in the generated content.
- Book successfully deploys with functional RAG chatbot, personalization, and translation features.
- Achieves bonus points by implementing subagents/skills, auth system, personalization, and Urdu translation.

## Governance

This constitution governs the development of the Physical AI & Humanoid Robotics course textbook. All contributions to the project must comply with these principles and standards. Any deviation from these principles must be documented and justified before implementation. New features or content must undergo verification against authoritative sources before inclusion.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Date of original adoption | **Last Amended**: 2025-12-10