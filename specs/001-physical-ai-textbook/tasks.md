---

description: "Task list for Physical AI & Humanoid Robotics Textbook implementation"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/` (Docusaurus), `.qwen/` (Qwen CLI tools)

---
## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan ensuring compliance with Physical AI & Humanoid Robotics Constitution
- [x] T002 Initialize Node.js and Python project with Docusaurus, FastAPI dependencies focusing on educational content quality
- [x] T003 [P] Configure verification tools for accuracy through authoritative sources
- [x] T004 [P] Install and configure Qwen CLI and Spec-Kit Plus tools for AI-driven content generation
- [x] T005 [P] Set up development environment with Python 3.11 and Node.js 18+

---
## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Setup database schema and migrations framework for user profiles and content management
- [x] T007 [P] Implement verification framework to ensure accuracy through authoritative sources (ROS 2, NVIDIA Isaac, Gazebo docs)
- [x] T008 [P] Setup Docusaurus framework for textbook structure with 4 modules and deployment to GitHub Pages
- [x] T009 Create base models/entities that all stories depend on in backend/src/models/
- [x] T010 Configure zero-plagiarism detection and content validation infrastructure
- [x] T011 Setup environment configuration management with quality gates
- [x] T012 [P] Configure Better-Auth for user authentication and profile management
- [x] T013 [P] Set up vector database (Qdrant Cloud) connection for RAG functionality
- [x] T014 [P] Configure Cohere for embeddings with multilingual support
- [x] T015 Initialize project configuration files and CI/CD pipeline for GitHub Pages

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Educational Content (Priority: P1) 🎯 MVP

**Goal**: Enable students and educators to access the comprehensive textbook content that covers 4 core modules (ROS 2, Gazebo & Unity, NVIDIA Isaac, and Vision-Language-Action)

**Independent Test**: Can be fully tested by accessing the Docusaurus-hosted textbook and verifying that all 4 modules with 3-4 chapters each (12-16 total) are available and contain the specified content on theoretical foundations, practical simulations, and code examples.

### Implementation for User Story 1

- [x] T016 [P] [US1] Create Textbook Module model in backend/src/models/module.py
- [x] T017 [P] [US1] Create Chapter model in backend/src/models/chapter.py
- [x] T018 [P] [US1] Create Book Content model in backend/src/models/content.py
- [x] T019 [US1] Implement Module service in backend/src/services/module_service.py
- [x] T020 [US1] Implement Chapter service in backend/src/services/chapter_service.py
- [x] T021 [US1] Create API endpoint GET /api/modules in backend/src/api/modules.py
- [x] T022 [US1] Create API endpoint GET /api/modules/{module_id}/chapters in backend/src/api/modules.py
- [x] T023 [US1] Create API endpoint GET /api/chapters/{chapter_id} in backend/src/api/chapters.py
- [x] T024 [P] [US1] Create basic Docusaurus sidebar structure for 4 modules in frontend/sidebars.js
- [x] T025 [P] [US1] Create placeholder content for Module 1 (ROS 2) in frontend/docs/ros2/
- [x] T026 [P] [US1] Create placeholder content for Module 2 (Gazebo & Unity) in frontend/docs/gazebo-unity/
- [x] T027 [P] [US1] Create placeholder content for Module 3 (NVIDIA Isaac) in frontend/docs/nvidia-isaac/
- [x] T028 [P] [US1] Create placeholder content for Module 4 (Vision-Language-Action) in frontend/docs/vla/
- [x] T029 [US1] Integrate Docusaurus with backend API to fetch and display module/chapter data
- [x] T030 [US1] Add theoretical foundations content to each chapter
- [x] T031 [US1] Add practical simulations content to each chapter
- [x] T032 [US1] Add code examples to each chapter with proper syntax highlighting
- [x] T033 [US1] Implement navigation between modules and chapters in Docusaurus interface
- [x] T034 [US1] Implement content validation to ensure all chapters meet 2000-4000 word requirement

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Ask Questions to RAG Chatbot (Priority: P1)

**Goal**: Allow students to query the textbook content and their selected text using an integrated RAG chatbot that understands the book's educational material

**Independent Test**: Can be fully tested by interacting with the RAG chatbot embedded in the Docusaurus site, asking questions about book content and verifying accurate answers.

### Implementation for User Story 2

- [x] T035 [P] [US2] Create RAG Index model in backend/src/models/rag_index.py
- [x] T036 [P] [US2] Create Interaction Log model in backend/src/models/interaction_log.py
- [x] T037 [US2] Implement RAG service to handle document indexing in backend/src/services/rag_service.py
- [x] T038 [US2] Implement RAG service to handle query processing in backend/src/services/rag_service.py
- [x] T039 [US2] Implement document chunking strategy (512-token chunks with 128 overlap) in backend/src/services/rag_service.py
- [x] T040 [US2] Create API endpoint POST /api/chat/query in backend/src/api/chat.py
- [x] T041 [US2] Implement interaction logging for query/response tracking in backend/src/services/logging_service.py
- [x] T042 [US2] Create embedding generation using Cohere embed-multilingual-v3.0 in backend/src/services/embedding_service.py
- [x] T043 [US2] Implement similarity search functionality in backend/src/services/rag_service.py
- [x] T044 [US2] Add source citation to RAG responses in backend/src/services/rag_service.py
- [x] T045 [US2] Integrate RAG chatbot component into Docusaurus pages in frontend/src/components/RAGChatbot.js
- [x] T046 [US2] Implement text selection and context passing to chatbot in frontend/src/components/ChapterContent.js
- [x] T047 [US2] Create UI for chat interface in Docusaurus site
- [x] T048 [US2] Implement caching for RAG responses to reduce API calls
- [x] T049 [US2] Add error handling for when RAG system cannot find an answer
- [x] T050 [US2] Implement graceful degradation when external services (Cohere, Qdrant) are unavailable

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - User Registration and Personalized Experience (Priority: P2)

**Goal**: Enable students to sign up for the platform with their software/hardware background information to receive personalized content adaptation

**Independent Test**: Can be fully tested by completing the registration flow with background information and then verifying that content adaptation occurs based on that background.

### Implementation for User Story 3

- [x] T051 [P] [US3] Enhance User Profile model with technical skills and background questionnaire in backend/src/models/user_profile.py
- [x] T052 [P] [US3] Create Subagent Configuration model in backend/src/models/subagent_config.py
- [x] T053 [P] [US3] Create Agent Skill model in backend/src/models/agent_skill.py
- [x] T054 [US3] Implement User service for registration with background info in backend/src/services/user_service.py
- [x] T055 [US3] Implement secure signup functionality via Better-Auth in backend/src/auth/signup.py
- [x] T056 [US3] Implement signup form with detailed technical skills collection (ROS 2, Unity, Python, etc.) in frontend/src/components/UserRegistration.js
- [x] T057 [US3] Create API endpoint POST /api/auth/signup in backend/src/api/auth.py
- [x] T058 [US3] Create API endpoint PUT /api/users/preferences in backend/src/api/users.py
- [x] T059 [US3] Implement content personalization based on user skills in backend/src/services/personalization_service.py
- [x] T060 [US3] Add content variants (beginner, intermediate, advanced) to chapter model in backend/src/models/chapter.py
- [x] T061 [US3] Implement difficulty-based content selection in backend/src/services/personalization_service.py
- [x] T062 [US3] Modify chapter API endpoint to return personalized content based on user profile in backend/src/api/chapters.py
- [x] T063 [US3] Implement preference saving and retrieval in frontend/src/contexts/UserContext.js
- [x] T064 [US3] Add UI controls for users to override content difficulty in frontend/src/components/ChapterDifficultySelector.js
- [x] T065 [US3] Create user profile dashboard to view and edit background information in frontend/src/pages/UserProfile.js
- [x] T066 [US3] Integrate user authentication with Docusaurus site using React context

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Translate Content to Urdu (Priority: P3)

**Goal**: Enable users to translate chapter content to Urdu for better comprehension in their native language

**Independent Test**: Can be fully tested by clicking the Urdu translation button in a chapter and verifying that the content is translated accurately.

### Implementation for User Story 4

- [x] T067 [P] [US4] Create Translation Data model in backend/src/models/translation_data.py
- [x] T068 [US4] Implement translation service using Qwen 1.5 Pro via subagent in backend/src/services/translation_service.py
- [x] T069 [US4] Create API endpoint POST /api/translate in backend/src/api/translation.py
- [x] T070 [US4] Implement content extraction for translation in backend/src/services/translation_service.py
- [x] T071 [US4] Add Urdu content to chapter model content_variants in backend/src/models/chapter.py
- [x] T072 [US4] Create Qwen CLI subagent configuration for Urdu translation in .qwen/subagents/translator.py
- [x] T073 [US4] Create Qwen CLI skill for translation validation in .qwen/skills/translation_validator.py
- [x] T074 [US4] Implement translation cache to avoid repeated API calls in backend/src/services/cache_service.py
- [x] T075 [US4] Add language preference to user profile in backend/src/models/user_profile.py
- [x] T076 [US4] Implement language switcher UI in frontend/src/components/LanguageSwitcher.js
- [x] T077 [US4] Add Urdu translation button at chapter start in frontend/src/components/ChapterControls.js
- [x] T078 [US4] Implement Urdu language support in Docusaurus configuration in frontend/docusaurus.config.js
- [x] T079 [US4] Add language-specific content rendering in frontend/src/components/ChapterContent.js
- [x] T080 [US4] Implement translation verification workflow in backend/src/services/translation_service.py
- [x] T081 [US4] Create content translation status tracking in backend/src/services/translation_service.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: Qwen CLI Subagents and Agent Skills Integration

**Goal**: Implement reusable intelligence through Qwen CLI Subagents and Agent Skills to generate dynamic content

**Independent Test**: Can be tested by using Qwen CLI to generate new content, translate chapters, or create code examples.

### Implementation for Subagents and Skills

- [x] T082 [P] Create Qwen CLI subagent for textbook content generation in .qwen/subagents/textbook_generator.py
- [x] T083 [P] Create Qwen CLI skill for code example generation in .qwen/skills/code_example_generator.py
- [x] T084 [P] Create Qwen CLI skill for content validation against official docs in .qwen/skills/content_validator.py
- [x] T085 [P] Create Qwen CLI skill for readability checking (Flesch-Kincaid ≤ 10) in .qwen/skills/readability_checker.py
- [x] T086 [P] Create Qwen CLI skill for diagram generation in .qwen/skills/diagram_generator.py
- [x] T087 Integrate content generation subagent with chapter creation process in backend/src/services/content_generation_service.py
- [x] T088 Implement validation of AI-generated content against authoritative sources in backend/src/services/validation_service.py
- [x] T089 Configure content generation pipeline to maintain quality standards in .qwen/config.json
- [x] T090 Document usage of Qwen CLI tools for content creators in docs/qwen-cli-usage.md

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T091 [P] Documentation updates in docs/
- [x] T092 Code cleanup and refactoring
- [x] T093 Performance optimization across all stories to ensure load times under 5s
- [x] T094 [P] Additional unit tests in backend/tests/ and frontend/tests/
- [x] T095 Security hardening for authentication and API endpoints
- [x] T096 Run quickstart.md validation for complete deployment flow
- [x] T097 [P] Performance testing with 100-500 simulated concurrent users
- [x] T098 Implement graceful degradation for external services (Cohere, Qdrant, BetterAuth)
- [x] T099 Add monitoring and logging for production deployment
- [x] T100 Deploy complete solution to GitHub Pages with backend on Render.com

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 (requires textbook content) but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 (requires textbook content) but should be independently testable
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Depends on US1 (requires textbook content) and US3 (user preferences) but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence