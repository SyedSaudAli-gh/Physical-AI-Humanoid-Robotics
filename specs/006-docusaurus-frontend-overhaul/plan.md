# Implementation Plan: Docusaurus Frontend - Complete UI Overhaul & Chatbot Integration

**Branch**: `006-docusaurus-frontend-overhaul` | **Date**: 2025-12-24 | **Spec**: [link to spec.md](spec.md)
**Input**: Feature specification from `/specs/006-docusaurus-frontend-overhaul/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The primary requirement is to completely overhaul the Docusaurus frontend UI with a modern, professional, book-style experience. This includes removing old backend chatbot components, implementing a new chatbot per Spec 4, creating module navigation cards, generating an AI-themed hero image, and ensuring full responsiveness. The technical approach involves removing legacy components, implementing new React components for the chatbot widget, updating the header and footer, and applying consistent styling throughout.

## Technical Context

**Language/Version**: JavaScript/TypeScript, React 18+, Docusaurus 3.x
**Primary Dependencies**: Docusaurus, React, React DOM, CSS Modules, Bootstrap/Styled Components
**Storage**: N/A (frontend only)
**Testing**: Jest, React Testing Library, Cypress for E2E testing
**Target Platform**: Web (all modern browsers, mobile, tablet, desktop)
**Project Type**: Web application
**Performance Goals**: <200ms initial load time, <100ms interactive elements response, 95% Lighthouse score
**Constraints**: Must be responsive across all device sizes, maintain accessibility standards, ensure zero old backend references remain
**Scale/Scope**: Single-page application, 10-15 pages/components, multiple responsive breakpoints

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

This project must comply with the Physical AI & Humanoid Robotics Course Textbook Constitution, ensuring:
- Accuracy through verification from primary or authoritative sources
- Clarity suitable for students with AI/computer science background
- Comprehensive coverage of theoretical and practical implementations
- Integration of Qwen CLI and Spec-Kit Plus tools
- Implementation of interactivity and personalization features
- Zero plagiarism in all generated content

**Constitution Compliance Status**: ✅ All requirements can be met with the proposed implementation approach. The UI overhaul will maintain clarity and progressive learning principles, while the new chatbot will enhance interactivity and personalization.

## Project Structure

### Documentation (this feature)

```text
specs/006-docusaurus-frontend-overhaul/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget/
│   │   ├── Header/
│   │   ├── Footer/
│   │   ├── HeroSection/
│   │   └── ModuleCard/
│   ├── pages/
│   ├── styles/
│   ├── utils/
│   └── assets/
│       └── images/
└── static/
    └── img/
```

**Structure Decision**: Web application with frontend directory containing React components for the Docusaurus site. This structure separates the frontend code while maintaining compatibility with Docusaurus conventions.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
