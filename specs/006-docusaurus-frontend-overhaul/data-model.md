# Data Model: Docusaurus Frontend Overhaul

## Entities

### ChatMessage
- **id**: string (unique identifier for the message)
- **content**: string (the text content of the message)
- **sender**: enum ['user', 'bot'] (indicates who sent the message)
- **timestamp**: datetime (when the message was created)
- **sources**: array of SourceReference (optional, for bot responses)

### SourceReference
- **title**: string (title of the source document)
- **url**: string (URL to the source)
- **excerpt**: string (relevant excerpt from the source)

### ModuleCard
- **id**: string (unique identifier for the module)
- **title**: string (title of the module)
- **description**: string (brief description of the module content)
- **chapterLink**: string (URL to the corresponding chapter)
- **icon**: string (optional icon identifier or path)

### NavigationItem
- **id**: string (unique identifier for the navigation item)
- **title**: string (display text for the navigation item)
- **url**: string (URL to navigate to)
- **children**: array of NavigationItem (optional, for sub-menus)

### CommunityLink
- **id**: string (unique identifier for the link)
- **name**: string (name of the platform, e.g., "GitHub", "LinkedIn")
- **url**: string (URL to the community page)
- **icon**: string (icon identifier or path)

## Relationships

- A ChatSession contains multiple ChatMessage entities
- A ChatMessage may reference multiple SourceReference entities (for bot responses)
- The HeroSection contains multiple ModuleCard entities
- The Header contains multiple NavigationItem entities
- The Footer contains multiple CommunityLink entities

## Validation Rules

- ChatMessage content must not be empty
- ModuleCard chapterLink must be a valid URL
- CommunityLink url must be a valid URL
- All timestamp fields must be in ISO 8601 format
- SourceReference fields are required when present in a bot response

## State Transitions (if applicable)

### ChatWidget States
- **Closed**: Initial state, only floating button visible
- **Opening**: Transition state when widget is expanding
- **Open**: Chat interface fully visible
- **Loading**: When waiting for response from backend
- **Error**: When there's an issue with the chat service