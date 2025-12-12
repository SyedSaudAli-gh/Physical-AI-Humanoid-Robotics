import json
from typing import Dict, Any, List
from sqlalchemy.orm import Session
from ..models.chapter import Chapter
from ..models.user_profile import UserProfile


class PersonalizationService:
    def __init__(self, db: Session):
        self.db = db

    def get_personalized_chapter_content(self, chapter_id: int, user_id: str) -> Dict[str, Any]:
        """Get personalized chapter content based on user profile"""

        # Get the chapter content
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")

        # Get user profile
        user_profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
        if not user_profile:
            raise ValueError(f"User profile with ID {user_id} not found")

        # Determine content based on user's experience level
        content = chapter.content

        # If chapter has content variants, adapt based on user's level
        if chapter.content_variants:
            try:
                variants = json.loads(chapter.content_variants)

                # Determine user's experience level (default to intermediate if not set)
                user_level = user_profile.experience_level or "intermediate"

                # Adjust content based on user level
                if user_level == "beginner":
                    content = variants.get("beginner", content)
                elif user_level == "advanced":
                    content = variants.get("advanced", content)
                else:
                    content = variants.get("intermediate", content)
            except json.JSONDecodeError:
                # If JSON parsing fails, use original content
                pass

        # Apply personalization based on user's technical skills
        personalization_applied = self._apply_technical_background_personalization(
            content, user_profile.technical_skills or [], user_profile.experience_level
        )

        # If user has specific technical interests, enhance relevant sections
        enhanced_content = self._enhance_content_for_skills(
            personalization_applied,
            user_profile.technical_skills or []
        )

        return {
            "id": chapter.id,
            "title": chapter.title,
            "content": enhanced_content,
            "original_content": chapter.content,
            "user_experience_level": user_profile.experience_level,
            "adapted_for_skills": user_profile.technical_skills
        }

    def _apply_technical_background_personalization(self, content: str, user_skills: List[str], user_level: str) -> str:
        """Adjust content based on user's technical background knowledge"""

        # Create a context for content adaptation based on user profile
        context = f"""
        User Level: {user_level}
        Technical Skills: {', '.join(user_skills)}
        """

        # If user has advanced skills, provide more concise but comprehensive explanations
        if user_level == "advanced" or len([skill for skill in user_skills if skill.lower() in
            ["ros2", "nvidia-isaac", "gazebo", "unity", "python", "cpp", "machine-learning", "computer-vision"]]) >= 3:

            return self._generate_advanced_content(content, context)

        # If user is intermediate, keep standard content but add appropriate examples
        elif user_level == "intermediate":
            return self._generate_intermediate_content(content, context)

        # If user is a beginner, add more explanations and examples
        else:
            return self._generate_beginner_content(content, context)

    def _generate_advanced_content(self, content: str, context: str) -> str:
        """Generate or modify content for advanced users"""
        # For advanced users, the content can be more concise with deeper technical details
        modified_content = content
        if "Advanced concepts" not in content:
            modified_content += "\n\n> Advanced Insight: For users with advanced experience in robotics and AI, consider exploring the performance implications of these techniques in real-time applications."

        return modified_content

    def _generate_intermediate_content(self, content: str, context: str) -> str:
        """Generate or modify content for intermediate users"""
        # For intermediate users, maintain a good balance between explanation and complexity
        modified_content = content
        if "Implementation details" not in content:
            modified_content += "\n\n> Implementation Note: As an intermediate user, you should be able to implement these concepts with the provided examples, but consider experimenting with different parameters to deepen your understanding."

        return modified_content

    def _generate_beginner_content(self, content: str, context: str) -> str:
        """Generate or modify content for beginners"""
        # For beginners, add more explanations and step-by-step guidance
        modified_content = content
        if "Beginner guidance" not in content:
            modified_content += "\n\n> Beginner Guidance: As a beginner in this field, focus on understanding the core concepts first before diving into implementation details. Try the examples in a simulation environment like Gazebo or Unity to get hands-on experience."

        return modified_content

    def _enhance_content_for_skills(self, content: str, user_skills: List[str]) -> str:
        """Enhance content with additional information related to user's technical skills"""
        enhanced_content = content

        # Add relevant examples based on user's technical skills
        for skill in user_skills:
            if skill.lower() in ["ros2", "ros"]:
                enhanced_content += f"\n\n> Integration with ROS2: This concept can be implemented in ROS2 by creating a node that interfaces with the system as shown in the previous examples."
            elif skill.lower() in ["unity"]:
                enhanced_content += f"\n\n> Unity Implementation: You can implement this concept in Unity by creating a GameObject with the required components and scripting as demonstrated in the examples."
            elif skill.lower() in ["gazebo"]:
                enhanced_content += f"\n\n> Gazebo Simulation: This approach can be simulated in Gazebo to test your implementation in a physics-accurate environment before applying to real robots."
            elif skill.lower() in ["nvidia-isaac"]:
                enhanced_content += f"\n\n> NVIDIA Isaac Integration: For NVIDIA Isaac users, consider using Isaac ROS packages to implement this concept with optimized GPU processing."
            elif skill.lower() in ["python"]:
                enhanced_content += f"\n\n> Python Implementation: All examples in this section can be implemented in Python using standard robotics libraries like ROS2 Python API."

        return enhanced_content