---
id: 6
title: Project-Cleanup-Tasks
stage: tasks
date: 2025-12-23
surface: agent
model: Qwen
feature: cleanup-old-backend
branch: 005-cleanup-old-backend
user: Yousuf Traders
command: /sp.tasks
labels: [cleanup, backend, project, tasks]
links:
  spec: specs/005-cleanup-old-backend/spec.md
  plan: specs/005-cleanup-old-backend/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/005-cleanup-old-backend/tasks.md
 - specs/005-cleanup-old-backend/spec.md
 - specs/005-cleanup-old-backend/plan.md
 - specs/005-cleanup-old-backend/research.md
 - specs/005-cleanup-old-backend/data-model.md
 - specs/005-cleanup-old-backend/quickstart.md
tests:
 - specs/005-cleanup-old-backend/tasks.md
---

## Prompt

$ARGUMENTS

## Response snapshot

I've successfully generated the tasks.md file for the project cleanup feature to remove the old backend folder. The tasks are organized in phases following the user stories from the specification:

1. Phase 1: Setup - Creating backup and verifying environment
2. Phase 2: Audit and Verification - Searching for all references to the old backend
3. Phase 3: User Story 1 - Remove Obsolete Backend Folder (P1)
4. Phase 4: User Story 2 - Verify No Broken References (P1)
5. Phase 5: User Story 3 - Preserve Active Backend Functionality (P1)
6. Phase 6: Validation and Documentation
7. Phase 7: Polish & Cross-Cutting Concerns

The tasks follow the required checklist format with:
- Markdown checkboxes
- Sequential task IDs (T001, T002, etc.)
- [P] markers for parallelizable tasks
- [US1], [US2], etc. labels for user story tasks
- Clear descriptions with file paths

Each user story phase includes:
- Goal of the user story
- Independent test criteria
- Acceptance scenarios
- Specific implementation tasks

Dependencies between phases are noted, and parallel execution opportunities are identified.

## Outcome

- ✅ Impact: Complete task breakdown for backend cleanup
- 🧪 Tests: Tasks include validation and verification requirements
- 📁 Files: tasks.md created with 35 specific tasks organized by phase
- 🔁 Next prompts: Ready for task implementation
- 🧠 Reflection: Tasks are organized by user story to enable systematic cleanup

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All tasks follow required format
- Prompt variant (if applicable): Standard /sp.tasks workflow
- Next experiment (smallest change to try): Begin with Phase 1 tasks