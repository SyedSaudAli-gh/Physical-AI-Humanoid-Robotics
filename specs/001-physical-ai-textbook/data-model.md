# Data Model: Physical AI & Humanoid Robotics Textbook

## User Profile
- **Fields:**
  - id (string, unique, required)
  - email (string, unique, required)
  - name (string, required)
  - auth_token (JWT, required)
  - technical_skills (array of strings, optional)
    - Examples: ["ROS 2", "Unity", "Python", "NVIDIA Isaac", "Gazebo", "C++"]
  - experience_level (enum: "beginner", "intermediate", "advanced", optional)
  - background_questionnaire (object, optional)
    - years_experience (number)
    - primary_language (string)
    - educational_background (string)
    - hardware_access (boolean)
  - preferences (object, optional)
    - preferred_language (string, default: "en")
    - chapter_difficulty_override (enum: "beginner", "intermediate", "advanced")
    - notification_preferences (object)
  - created_at (timestamp, required)
  - updated_at (timestamp, required)

**Relationships:** One User Profile to many Interaction Logs

## Textbook Module
- **Fields:**
  - id (string, unique, required)
  - name (string, required)
    - Examples: "The Robotic Nervous System (ROS 2)", "The Digital Twin (Gazebo & Unity)"
  - description (string, required)
  - order_index (number, required)
  - learning_objectives (array of strings, required)
  - prerequisites (array of strings, optional)
  - created_at (timestamp, required)
  - updated_at (timestamp, required)

## Chapter
- **Fields:**
  - id (string, unique, required)
  - module_id (string, required, references Textbook Module)
  - title (string, required)
  - content (string, required, 2000-4000 words)
  - content_variants (object, optional)
    - beginner (string, simplified content)
    - intermediate (string, default content)
    - advanced (string, detailed content)
    - urdu (string, translated content)
  - order_index (number, required)
  - learning_outcomes (array of strings, required)
  - code_examples (array of objects, required)
    - language (string)
    - code (string)
    - description (string)
  - exercises (array of objects, optional)
    - type (string)
    - content (string)
    - difficulty (enum: "beginner", "intermediate", "advanced")
  - created_at (timestamp, required)
  - updated_at (timestamp, required)

**Relationships:** One Textbook Module to many Chapters

## Book Content
- **Fields:**
  - id (string, unique, required)
  - chapter_id (string, required, references Chapter)
  - content_type (enum: "text", "code", "diagram", "exercise", "summary")
  - content (string, required)
  - version (number, required, default: 1)
  - language (string, required, default: "en")
  - difficulty_level (enum: "beginner", "intermediate", "advanced")
  - created_at (timestamp, required)
  - updated_at (timestamp, required)

## Translation Data
- **Fields:**
  - id (string, unique, required)
  - source_content_id (string, required, references Book Content)
  - target_language (string, required)
  - translated_content (string, required)
  - verification_status (enum: "pending", "verified", "needs_revision", default: "pending")
  - verified_by (string, optional)
  - created_at (timestamp, required)
  - updated_at (timestamp, required)

## RAG Index
- **Fields:**
  - id (string, unique, required)
  - content_id (string, required, references Book Content)
  - embedding (array of numbers, required)
  - chunk_text (string, required)
  - chunk_metadata (object, required)
    - chapter_id (string)
    - module_id (string)
    - content_type (string)
    - difficulty_level (string)
  - created_at (timestamp, required)

## Interaction Log
- **Fields:**
  - id (string, unique, required)
  - user_id (string, required, references User Profile)
  - interaction_type (enum: "chat_query", "chapter_view", "translation", "personalization", "exercise", "code_run")
  - content_id (string, optional, references Book Content or Chapter)
  - query_text (string, optional)
  - response_text (string, optional)
  - selected_text (string, optional, for RAG queries)
  - personalization_applied (object, optional)
    - original_difficulty (string)
    - applied_difficulty (string)
    - content_changed (boolean)
  - timestamp (timestamp, required)
  - session_id (string, required)

## Subagent Configuration
- **Fields:**
  - id (string, unique, required)
  - name (string, required)
  - purpose (string, required)
    - Examples: "content_generation", "translation", "code_example", "diagram_generation"
  - parameters (object, optional)
  - created_at (timestamp, required)
  - updated_at (timestamp, required)

## Agent Skill
- **Fields:**
  - id (string, unique, required)
  - name (string, required)
  - description (string, required)
  - command (string, required)
  - usage_examples (array of strings, optional)
  - created_at (timestamp, required)
  - updated_at (timestamp, required)