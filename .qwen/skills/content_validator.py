"""
Qwen CLI Skill for Content Validation Against Official Documentation
This module provides functionality to validate generated content against authoritative sources
like ROS 2, NVIDIA Isaac, Gazebo, and Unity documentation.
"""
from typing import Dict, Any, List
import re
import requests
from urllib.parse import urljoin
from ..verification_framework import VerificationFramework

class ContentValidatorSkill:
    """
    Skill for validating content against official documentation
    """
    
    def __init__(self):
        self.verification_framework = VerificationFramework()
        self.authoritative_sources = {
            'ros2': 'https://docs.ros.org/',
            'nvidia-isaac': 'https://docs.nvidia.com/isaac/',
            'gazebo': 'http://gazebosim.org/',
            'unity': 'https://docs.unity3d.com/',
            'cohere': 'https://docs.cohere.com/',
            'qdrant': 'https://qdrant.tech/documentation/',
            'better-auth': 'https://better-auth.com/docs',
            'docusaurus': 'https://docusaurus.io/docs',
            'fastapi': 'https://fastapi.tiangolo.com/'
        }
    
    def validate_content_accuracy(self, content: str, source_type: str) -> Dict[str, Any]:
        """
        Validate content against authoritative sources
        """
        # Use the verification framework to check content accuracy
        is_accurate, issues, verified_sources = self.verification_framework.verify_content_accuracy(
            content, source_type
        )
        
        # Additional checks specific to this skill
        technical_accuracy_issues = self.verification_framework.check_technical_accuracy(
            content, source_type
        )
        
        return {
            "is_accurate": is_accurate,
            "issues": issues,
            "technical_accuracy_issues": technical_accuracy_issues,
            "verified_sources": verified_sources,
            "source_type": source_type,
            "validation_score": self._calculate_validation_score(issues, technical_accuracy_issues)
        }
    
    def _calculate_validation_score(self, issues: List[str], technical_issues: List[str]) -> float:
        """
        Calculate a validation score based on issues found
        """
        total_issues = len(issues) + len(technical_issues)
        # Score is 1.0 for no issues, decreases with more issues
        # Minimum score is 0.0
        score = max(0.0, 1.0 - (total_issues * 0.1))
        return round(score, 2)
    
    def validate_against_documentation(self, content: str, documentation_url: str) -> Dict[str, Any]:
        """
        Validate content specifically against documentation at the given URL
        """
        try:
            # In a real implementation, this would fetch documentation and compare
            # For this example, we'll just validate using our framework
            source_type = self._get_source_type_from_url(documentation_url)
            
            return self.validate_content_accuracy(content, source_type)
            
        except Exception as e:
            return {
                "is_accurate": False,
                "issues": [f"Error validating against documentation: {str(e)}"],
                "technical_accuracy_issues": [],
                "verified_sources": [],
                "source_type": "unknown",
                "validation_score": 0.0
            }
    
    def _get_source_type_from_url(self, url: str) -> str:
        """
        Determine source type from documentation URL
        """
        for source_type, base_url in self.authoritative_sources.items():
            if base_url in url:
                return source_type
        return "unknown"

def validate_content_accuracy(content: str, source_type: str) -> Dict[str, Any]:
    """
    Main function to validate content accuracy
    This would be called by the Qwen CLI
    """
    validator = ContentValidatorSkill()
    result = validator.validate_content_accuracy(content, source_type)
    return result

def validate_against_documentation(content: str, documentation_url: str) -> Dict[str, Any]:
    """
    Main function to validate content against specific documentation
    This would be called by the Qwen CLI
    """
    validator = ContentValidatorSkill()
    result = validator.validate_against_documentation(content, documentation_url)
    return result

if __name__ == "__main__":
    print("Content Validation Skill initialized")
    
    # Example usage
    sample_content = """
    This is sample content about ROS 2 nodes and communication.
    ROS 2 provides the communication infrastructure for robotics applications.
    Nodes are the fundamental execution units in ROS 2.
    """
    
    result = validate_content_accuracy(sample_content, "ros2")
    print(f"Validation result: {result}")