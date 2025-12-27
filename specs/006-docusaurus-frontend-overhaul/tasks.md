# Tasks: Docusaurus Frontend - Complete UI Overhaul & Chatbot Integration

## Feature Overview

This feature implements the complete UI overhaul of the Docusaurus frontend with a modern, professional, book-style experience. It includes removing old backend chatbot components, implementing a new chatbot per Spec 4, creating module navigation cards, generating an AI-themed hero image, and ensuring full responsiveness. The tasks will ensure all requirements from the frontend enforcement fix are properly implemented.

## Dependencies

User stories can be implemented in parallel after foundational tasks are completed. User Story 2 (Chatbot) has a dependency on the backend API endpoint being available. User Story 5 (Responsiveness) affects all other stories and should be considered throughout development.

## Parallel Execution Examples

- **US1 (UI Overhaul)** and **US2 (Chatbot)** can be developed in parallel after foundational tasks are completed
- **US3 (Module Navigation)** and **US4 (Footer/Links)** can be developed in parallel
- **US5 (Responsiveness)** needs to be considered throughout all other user stories

## Implementation Strategy

- **MVP Scope**: User Story 1 (Basic UI Overhaul) + User Story 2 (Basic Chatbot) + User Story 5 (Responsiveness) for core functionality
- **Incremental Delivery**: Each user story provides independent value and can be deployed separately
- **Testing Approach**: Each component will be audited and fixed individually before integration

---

## Phase 1: Setup

- [ ] T001 Set up development environment and verify project structure
- [ ] T002 Install required dependencies (React DOM, CSS Modules, Bootstrap/Styled Components)
- [ ] T003 Configure development environment and build process
- [ ] T004 Set up testing framework (Jest, React Testing Library, Cypress)

---

## Phase 2: Foundational Tasks - Frontend Audit & Cleanup

- [X] T010 [P] Scan entire frontend folder for old backend references
- [X] T011 [P] List all references to old backend, legacy chatbot components, unused API calls, broken imports or paths
- [X] T012 [P] Physically delete old backend-related files
- [X] T013 [P] Physically delete unused chatbot components
- [X] T014 [P] Remove dead imports and API references
- [X] T015 [P] Update package.json to remove unused dependencies
- [X] T016 [P] Update configuration files that reference the old backend
- [X] T017 [P] Remove Urdu translation functionality files that are not part of current spec
- [X] T018 [P] Clean up unused API endpoints in constants and services
- [X] T019 [P] Verify zero old backend references remain in frontend codebase

---

## Phase 3: User Story 1 - Recreate and Render Navbar and Footer (Priority: P1)

**Story Goal**: Recreate Header/Navbar as a visible component and ensure it renders on all pages. Recreate Footer with GitHub, LinkedIn, Facebook, Twitter links and verify Navbar and Footer appear in the UI.

**Independent Test Criteria**: The Navbar and Footer can be tested by navigating to different pages and verifying they are visible and all links work correctly.

**Acceptance Scenarios**:
1. Given I am on any page of the site, When I look at the top, Then I see a visible Navbar with proper links
2. Given I am on any page of the site, When I scroll to the bottom, Then I see a visible Footer with all required links
3. Given I am using a mobile device, When I access the site, Then I see a responsive Navbar with hamburger menu

- [X] T020 [US1] Create Header component with logo, navigation menu, responsive hamburger menu
- [X] T021 [US1] Style Header with clean, modern design
- [X] T022 [US1] Implement responsive behavior for mobile devices
- [X] T023 [US1] Create Footer component with GitHub, LinkedIn, Facebook, Twitter links
- [X] T024 [US1] Add copyright information to Footer
- [X] T025 [US1] Style Footer with clean, modern design
- [X] T026 [US1] Ensure Header and Footer render on all pages
- [X] T027 [US1] Test all Navbar links navigate to correct pages
- [X] T028 [US1] Test all Footer links navigate to correct external URLs

---

## Phase 4: User Story 2 - Re-implement Chatbot UI per Spec 4 (Priority: P1)

**Story Goal**: Implement floating chat widget that strictly follows Spec 4 requirements with visible chat button, expandable panel, and /api/chat POST integration.

**Independent Test Criteria**: The chatbot functionality can be tested by opening the chat widget, sending messages, and verifying they connect to the backend.

**Acceptance Scenarios**:
1. Given I am viewing content on the site, When I see the page, Then I see a visible floating chat button
2. Given I click the floating chat button, When the widget opens, Then I see an expandable panel
3. Given I send a message in the chat, When I submit it, Then it connects to /api/chat endpoint
4. Given the chat service is experiencing issues, When I try to interact with the chatbot, Then I see appropriate loading or error states

- [X] T030 [US2] Create ChatWidget component with floating button as per Spec 4
- [X] T031 [US2] Implement expandable panel functionality as per Spec 4
- [X] T032 [US2] Implement API service for connecting to POST /api/chat endpoint
- [X] T033 [US2] Create ChatMessage entity component to display messages
- [X] T034 [US2] Create SourceReference component to display response sources
- [X] T035 [US2] Implement loading and error states for chat interactions
- [X] T036 [US2] Ensure no extra or experimental chatbot features are included
- [X] T037 [US2] Test chatbot functionality connects to backend correctly

---

## Phase 5: User Story 3 - Fix Hero Section with Module Cards (Priority: P2)

**Story Goal**: Ensure Hero section renders correctly with module cards linking to chapters and a Physical AI & Humanoid Robotics hero image.

**Independent Test Criteria**: The Hero section can be tested by viewing the homepage and verifying all elements render correctly and links work.

**Acceptance Scenarios**:
1. Given I am on the homepage, When I view the Hero section, Then I see it renders correctly
2. Given I am on the homepage, When I view the Hero section, Then I see module cards linking to chapters
3. Given I am on the homepage, When I view the Hero section, Then I see the Physical AI & Humanoid Robotics hero image

- [X] T040 [US3] Ensure Hero section renders correctly
- [X] T041 [US3] Create module cards linking to chapters
- [X] T042 [US3] Generate and place Physical AI & Humanoid Robotics hero image
- [X] T043 [US3] Implement navigation functionality for module cards
- [X] T044 [US3] Style Hero section with modern, clean design
- [X] T045 [US3] Test module card links navigate to correct chapters
- [X] T046 [US3] Verify hero image displays correctly

---

## Phase 6: User Story 4 - Apply Clean, Modern Styling (Priority: P2)

**Story Goal**: Apply consistent typography, spacing, colors and fix layout breaks and rendering issues to ensure professional book-style UI.

**Independent Test Criteria**: The styling can be tested by viewing different pages and verifying consistent typography, spacing, and colors.

**Acceptance Scenarios**:
1. Given I am viewing any page, When I look at the typography, Then I see consistent, professional styling
2. Given I am viewing any page, When I look at the layout, Then I see proper spacing and no rendering issues
3. Given I am viewing any page, When I look at the UI, Then I see a professional book-style design

- [X] T050 [US4] Apply consistent typography across all components
- [X] T051 [US4] Apply consistent spacing across all components
- [X] T052 [US4] Apply consistent color palette across all components
- [X] T053 [US4] Fix layout breaks and rendering issues
- [X] T054 [US4] Ensure professional book-style UI implementation
- [X] T055 [US4] Verify all components follow the styling guidelines
- [X] T056 [US4] Test UI improvements across different pages

---

## Phase 7: User Story 5 - Ensure Full Responsiveness (Priority: P1)

**Story Goal**: Verify mobile, tablet, desktop layouts and fix overflow and visibility issues.

**Independent Test Criteria**: Responsiveness can be tested by viewing the site on different screen sizes and verifying all elements adapt appropriately.

**Acceptance Scenarios**:
1. Given I am using a mobile device, When I access the site, Then all elements display correctly with proper layout
2. Given I am using a tablet device, When I access the site, Then all elements display correctly with proper layout
3. Given I am using a desktop device, When I access the site, Then all elements display correctly with proper layout

- [X] T060 [US5] Verify mobile layout and fix any issues
- [X] T061 [US5] Verify tablet layout and fix any issues
- [X] T062 [US5] Verify desktop layout and fix any issues
- [X] T063 [US5] Fix overflow issues on all screen sizes
- [X] T064 [US5] Fix visibility issues on all screen sizes
- [X] T065 [US5] Test responsive behavior of all components
- [X] T066 [US5] Ensure all functionality works on different viewport sizes

---

## Phase 8: Polish & Cross-Cutting Concerns

- [X] T070 [P] Identify UI/runtime errors in the application
- [X] T071 [P] Fix all identified UI/runtime errors until clean build is achieved
- [X] T072 [P] Ensure chatbot matches Spec 4 exactly with no extra features
- [X] T073 [P] Verify all links work correctly
- [X] T074 [P] Ensure UI is visually improved as required
- [X] T075 [P] Verify frontend builds and runs successfully
- [X] T076 [P] Conduct final integration testing of all components
- [X] T077 [P] Run accessibility audit and fix any issues
- [X] T078 [P] Run performance audit and optimize as needed
- [X] T079 [P] Verify all success criteria from frontend enforcement fix are met
- [X] T080 [P] Prepare production build and deployment configuration