# Qwen CLI Tools for Content Creators

This document explains how to use the Qwen CLI tools for generating content for the Physical AI & Humanoid Robotics Textbook.

## Setup

1. Ensure you have the Qwen CLI installed and configured:
   ```bash
   pip install qwen-cli
   qwen-cli configure
   ```

2. Set up your API keys and authentication as required.

## Available Subagents

### Textbook Content Generator

Generate textbook chapters with proper formatting and educational structure:

```bash
qwen-cli subagent textbook generate-content \
  --module "ROS 2" \
  --title "Understanding ROS 2 Nodes" \
  --topic "ROS 2 Nodes and Communication" \
  --target-audience "undergraduate_students" \
  --word-count 3000
```

## Available Skills

### 1. Code Example Generator

Generate code examples for specific topics and programming languages:

```bash
qwen-cli skill code-example generate \
  --topic "Subscriber" \
  --language "Python" \
  --complexity "intermediate" \
  --module "ROS 2"
```

### 2. Content Validator

Validate content against authoritative sources:

```bash
qwen-cli skill content-validator validate \
  --source-type "ros2" \
  --content-file "chapter_content.txt"
```

### 3. Readability Checker

Check content readability against Flesch-Kincaid grade level ≤ 10:

```bash
qwen-cli skill readability check \
  --content-file "chapter_content.txt" \
  --target-level 10
```

### 4. Diagram Generator

Generate diagrams to illustrate concepts:

```bash
qwen-cli skill diagram generate \
  --topic "ROS Node Communication" \
  --type "sequence" \
  --description "Shows how nodes communicate in ROS 2"
```

## Best Practices

1. **Consistency**: Always use consistent terminology across chapters
2. **Structure**: Follow the required textbook structure with Learning Outcomes, Theoretical Foundations, Practical Simulations, Code Examples, and Chapter Summary
3. **Technical Accuracy**: Validate all technical content against official documentation
4. **Readability**: Ensure all content meets the target Flesch-Kincaid grade level of 10 or below
5. **Originality**: All content must pass plagiarism checks with ≥95% originality

## Quality Standards

- Content must include required sections: Learning Outcomes, Theoretical Foundations, Practical Simulations, Code Examples, Chapter Summary
- At least 1 code example per chapter
- At least 1 diagram per chapter
- Flesch-Kincaid grade level ≤ 10
- Minimum quality score of 0.7 after validation
- Content must be original (≥95% originality)

## Integration with Textbook Platform

Generated content will automatically be:
- Validated against authoritative sources (ROS 2, NVIDIA Isaac, Gazebo, Unity docs)
- Checked for readability
- Translated to Urdu (if applicable)
- Personalized for different difficulty levels (beginner, intermediate, advanced)
- Integrated into the Docusaurus-based textbook platform

## Troubleshooting

If you encounter issues with content generation:

1. Check your API keys and authentication
2. Verify the topic and module parameters are valid
3. Ensure your content meets the minimum requirements
4. Check the logs for detailed error messages