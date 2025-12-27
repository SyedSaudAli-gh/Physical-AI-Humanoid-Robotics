# Quickstart Guide: Docusaurus Frontend Overhaul

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- Basic knowledge of React and Docusaurus

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the frontend directory**
   ```bash
   cd frontend
   ```

3. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

4. **Start the development server**
   ```bash
   npm run start
   # or
   yarn start
   ```

5. **Open your browser to**
   ```
   http://localhost:3000
   ```

## Key Directories and Files

- `src/components/` - React components for the UI overhaul
- `src/components/ChatWidget/` - Chatbot widget implementation
- `src/components/Header/` - Header component with navigation
- `src/components/Footer/` - Footer component with links
- `src/components/HeroSection/` - Hero section with module cards
- `src/styles/` - CSS modules and global styles
- `static/img/` - Static images including the hero image

## Implementation Tasks

### 1. Remove Old Backend Components
- Locate and remove all old chatbot components, imports, and API references
- Check for dependencies in package.json that are no longer needed
- Update any configuration files that reference the old backend

### 2. Implement ChatWidget Component
- Create the floating chat button component
- Implement the expandable chat window
- Connect to rag-backend POST /api/chat endpoint
- Add text selection functionality
- Implement loading and error states
- Display response sources

### 3. Update Header Component
- Design responsive header with logo
- Implement navigation menu
- Add responsive hamburger menu for mobile
- Apply modern, clean styling

### 4. Create Hero Section
- Implement module navigation cards with chapter links
- Generate and place Physical AI/Robotics themed hero image
- Add clear call-to-action for book exploration

### 5. Create Footer Component
- Add GitHub repository link
- Include community social links (LinkedIn, Facebook, Twitter)
- Add copyright section
- Ensure responsive design

### 6. Apply Consistent Styling
- Implement consistent typography across all components
- Apply consistent spacing using CSS modules
- Define and use a consistent color palette
- Ensure modern, clean design throughout

### 7. Ensure Responsiveness
- Test all components on small, medium, and large screens
- Implement responsive breakpoints
- Verify mobile navigation works properly
- Test touch interactions on mobile devices

## API Integration

The chatbot widget connects to the RAG backend using the following endpoint:
- **POST** `/api/chat`
- Request body: `{ "message": "user query", "context": "selected text context" }`
- Response: `{ "response": "AI response", "sources": [...] }`

## Testing

1. **Unit Tests**
   ```bash
   npm run test
   # or
   yarn test
   ```

2. **End-to-End Tests**
   ```bash
   npm run e2e
   # or
   yarn e2e
   ```

3. **Manual Testing Checklist**
   - [ ] All navigation links work correctly
   - [ ] Chatbot opens and sends/receives messages
   - [ ] Text selection works with chatbot
   - [ ] Responsive design works on all screen sizes
   - [ ] All social links in footer open correctly
   - [ ] Hero image displays properly
   - [ ] Module cards link to correct chapters

## Deployment

To build the production version:
```bash
npm run build
# or
yarn build
```

The built site will be in the `build/` directory and can be served using any static hosting service.