---
id: 4
title: Project-Cleanup-Spec
stage: spec
date: 2025-12-23
surface: agent
model: Qwen
feature: cleanup-old-backend
branch: 005-cleanup-old-backend
user: Yousuf Traders
command: /sp.specify
labels: [cleanup, backend, project, specification]
links:
  spec: specs/005-cleanup-old-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/005-cleanup-old-backend/spec.md
 - specs/005-cleanup-old-backend/checklists/requirements.md
tests:
 - specs/005-cleanup-old-backend/checklists/requirements.md
---

## Prompt

$ARGUMENTS

## Response snapshot

I've created a feature specification for the project cleanup task to remove the old backend folder. The specification includes:

- Clear user stories with priorities for the cleanup operation
- Functional requirements that detail what needs to be removed and what needs to be preserved
- Success criteria that are measurable and technology-agnostic
- Acceptance scenarios for each user story
- Edge cases to consider during the cleanup

The specification ensures that:
- The obsolete /backend directory is completely removed
- All references to the old backend are identified and removed
- The active rag-backend remains fully functional
- Project continues to run without errors after cleanup
- Git history is preserved with a clean commit

## Outcome

- ✅ Impact: Created comprehensive specification for backend cleanup
- 🧪 Tests: Created checklist to validate specification quality
- 📁 Files: spec.md and requirements.md created in feature directory
- 🔁 Next prompts: Ready for /sp.plan or /sp.tasks
- 🧠 Reflection: Specification clearly defines scope and constraints for cleanup operation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All specification quality checks passed
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): Proceed with planning phase