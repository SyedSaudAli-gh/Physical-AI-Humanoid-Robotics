from sqlalchemy.orm import Session
from typing import Dict, Any, List
from ..models.chapter import Chapter
from ...qwen.skills.content_validator import validate_content_accuracy
from ..content_validation import ContentValidator
from ..services.degradation_service import service_status
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ValidationService:
    """
    Service for validating AI-generated content against authoritative sources
    """
    
    def __init__(self, db: Session):
        self.db = db
        self.content_validator = ContentValidator()
    
    def validate_chapter_content(self, chapter_id: str, source_type: str = "mixed") -> Dict[str, Any]:
        """
        Validate a chapter's content against authoritative sources
        """
        try:
            # Get the chapter from the database
            chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
            if not chapter:
                return {
                    "success": False,
                    "message": f"Chapter with ID {chapter_id} not found"
                }
            
            # Validate against authoritative sources using the Qwen skill
            content_validation_result = validate_content_accuracy(
                chapter.content, 
                source_type
            )
            
            # Run additional validation with our internal validator
            chapter_validation = self.content_validator.validate_content(
                chapter.content, 
                content_type="chapter"
            )
            
            # Combine results
            combined_result = {
                "chapter_id": chapter_id,
                "title": chapter.title,
                "content_validation": content_validation_result,
                "internal_validation": chapter_validation,
                "overall_validation_score": self._calculate_combined_score(
                    content_validation_result, 
                    chapter_validation
                ),
                "is_valid": (
                    content_validation_result["is_accurate"] and 
                    chapter_validation["is_valid"]
                )
            }
            
            # Log validation results
            if combined_result["is_valid"]:
                logger.info(f"Chapter {chapter_id} passed all validations")
            else:
                logger.warning(f"Chapter {chapter_id} failed validation")
                
            return {
                "success": True,
                "validation_result": combined_result
            }
            
        except Exception as e:
            logger.error(f"Error validating chapter {chapter_id}: {str(e)}")
            return {
                "success": False,
                "message": f"Error validating chapter: {str(e)}"
            }
    
    def validate_content_batch(self, chapter_ids: List[str], source_type: str = "mixed") -> Dict[str, Any]:
        """
        Validate multiple chapters at once
        """
        results = []
        success_count = 0
        
        for chapter_id in chapter_ids:
            result = self.validate_chapter_content(chapter_id, source_type)
            if result["success"]:
                success_count += 1
                results.append(result["validation_result"])
            else:
                results.append({
                    "chapter_id": chapter_id,
                    "error": result["message"],
                    "is_valid": False
                })
        
        return {
            "total_validated": len(chapter_ids),
            "successful_validations": success_count,
            "failed_validations": len(chapter_ids) - success_count,
            "results": results
        }
    
    def _calculate_combined_score(self, content_validation_result: Dict[str, Any], 
                                 internal_validation: Dict[str, Any]) -> float:
        """
        Calculate an overall validation score combining multiple validation methods
        """
        # Get scores from both validation methods
        content_accuracy_score = content_validation_result.get("validation_score", 0.0)
        internal_score = 1.0 if internal_validation.get("is_valid", False) else 0.0
        
        # Calculate weighted average (adjust weights as needed)
        combined_score = (content_accuracy_score * 0.7) + (internal_score * 0.3)
        
        return round(combined_score, 2)
    
    def validate_new_content(self, content: str, title: str, source_type: str = "mixed") -> Dict[str, Any]:
        """
        Validate new content before it's saved to the database
        """
        # Run content validation against authoritative sources
        content_validation = validate_content_accuracy(content, source_type)
        
        # Run internal content validation
        internal_validation = self.content_validator.validate_content(
            content, 
            content_type="new_content"
        )
        
        # Check for readability requirements
        readability_result = self._check_readability(content)
        
        result = {
            "title": title,
            "content_validation": content_validation,
            "internal_validation": internal_validation,
            "readability_check": readability_result,
            "overall_validation_score": self._calculate_combined_score(
                content_validation, 
                internal_validation
            ),
            "is_valid": (
                content_validation["is_accurate"] and 
                internal_validation["is_valid"] and
                readability_result["is_readable"]
            )
        }
        
        return result
    
    def _check_readability(self, content: str) -> Dict[str, Any]:
        """
        Check if content meets readability requirements (Flesch-Kincaid ≤ 10)
        """
        # Import the readability checker from the Qwen skills
        from ...qwen.skills.readability_checker import check_readability
        return check_readability(content)

# Example usage
if __name__ == "__main__":
    print("Validation Service created")