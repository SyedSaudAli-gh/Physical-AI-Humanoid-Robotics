"""
Unit tests for the backend services of the Physical AI & Humanoid Robotics Textbook project

This file contains unit tests for critical services and functionality
"""
import unittest
from unittest.mock import Mock, MagicMock, patch
from fastapi.testclient import TestClient
from src.main import app
from src.services.user_service import UserService
from src.services.rag_service import RAGService
from src.services.translation_service import TranslationService
from src.models.user_profile import User

class TestUserService(unittest.TestCase):
    """
    Unit tests for the UserService class
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.mock_db = Mock()
        self.user_service = UserService(self.mock_db)
    
    @patch('src.auth.auth_service.register_user')
    def test_register_user_with_background(self, mock_register_user):
        """Test registering a user with background information."""
        # Mock the return value of the register_user function
        mock_user_data = Mock()
        mock_user_data.id = "test_id"
        mock_user_data.email = "test@example.com"
        mock_user_data.name = "Test User"
        mock_user_data.technical_skills = ["Python", "ROS 2"]
        mock_user_data.experience_level = "intermediate"
        mock_user_data.background_questionnaire = {"years_experience": 3}
        mock_user_data.preferences = {}
        mock_register_user.return_value = mock_user_data
        
        # Test data
        from src.services.user_service import UserRegistrationRequest
        user_req = UserRegistrationRequest(
            email="test@example.com",
            password="securepassword",
            name="Test User",
            technical_skills=["Python", "ROS 2"],
            experience_level="intermediate",
            background_questionnaire={"years_experience": 3}
        )
        
        # Call the method
        result = self.user_service.register_user_with_background(user_req)
        
        # Assertions
        self.assertEqual(result['email'], 'test@example.com')
        self.assertEqual(result['name'], 'Test User')
        self.assertIn('Python', result['technical_skills'])
        self.assertEqual(result['experience_level'], 'intermediate')
        mock_register_user.assert_called_once()


class TestRAGService(unittest.TestCase):
    """
    Unit tests for the RAGService class
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.mock_db = Mock()
        self.rag_service = RAGService(self.mock_db)
    
    def test_generate_response_with_context(self):
        """Test generating response with context."""
        # Test the internal method that generates responses
        query = "What are ROS 2 nodes?"
        context_texts = ["ROS 2 nodes are the fundamental execution units..."]
        selected_text = "nodes are the fundamental execution units"
        
        response = self.rag_service._generate_response_with_context(
            query, context_texts, selected_text
        )
        
        # Check that the response contains the query and context
        self.assertIn("ROS 2 nodes", response)
        self.assertIsInstance(response, str)


class TestTranslationService(unittest.TestCase):
    """
    Unit tests for the TranslationService class
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.mock_db = Mock()
        self.translation_service = TranslationService(self.mock_db)
    
    def test_get_urdu_translation(self):
        """Test the Urdu translation functionality."""
        content = "This is a test for translation."
        
        # Since _get_urdu_translation is a placeholder implementation,
        # we're testing that it returns a string containing the original content
        result = self.translation_service._get_urdu_translation(content)
        
        self.assertIsInstance(result, str)
        self.assertIn("PLACEHOLDER URDU TRANSLATION", result)
        self.assertIn(content, result)
    
    def test_translate_content_urdu(self):
        """Test translating content to Urdu."""
        content = "Introduction to robotics"
        
        result = self.translation_service.translate_content(content, target_language="ur")
        
        self.assertIsInstance(result, str)
        self.assertIn("[PLACEHOLDER URDU TRANSLATION]", result)


class TestAPIEndpoints(unittest.TestCase):
    """
    Unit tests for API endpoints
    """
    
    def setUp(self):
        self.client = TestClient(app)
    
    def test_health_endpoint(self):
        """Test the health check endpoint."""
        response = self.client.get("/health")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "healthy"})
    
    def test_root_endpoint(self):
        """Test the root endpoint."""
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        
        # The response should contain the welcome message
        data = response.json()
        self.assertIn("message", data)


class TestContentValidation(unittest.TestCase):
    """
    Unit tests for content validation functionality
    """
    
    def test_content_word_count_validation(self):
        """Test content validation based on word count."""
        from src.content_validation import ContentValidator
        
        validator = ContentValidator()
        
        # Test with content that meets requirements (>2000 words)
        long_content = "word " * 2100
        result = validator.validate_word_count(long_content, min_words=2000, max_words=4000)
        
        self.assertTrue(result['isValid'])
        self.assertEqual(result['wordCount'], 2100)
        
        # Test with content that is too short (<2000 words)
        short_content = "word " * 100  # Only 100 words
        result = validator.validate_word_count(short_content, min_words=2000, max_words=4000)
        
        self.assertFalse(result['isValid'])
        self.assertEqual(result['wordCount'], 100)
        self.assertIn("too short", result['message'])


# Integration tests - testing multiple components working together
class TestIntegration(unittest.TestCase):
    """
    Integration tests to ensure components work together properly
    """
    
    @patch('src.services.rag_service.RAGService')
    def test_rag_service_initialization(self, mock_rag_service):
        """Test that RAG service initializes properly with database session."""
        # Create a mock database session
        mock_db = Mock()
        
        # Initialize RAG service with mock DB
        rag_service = RAGService(mock_db)
        
        # Verify it was initialized correctly
        self.assertIs(rag_service.db, mock_db)


if __name__ == '__main__':
    print("Running unit tests for Physical AI & Humanoid Robotics Textbook backend...")
    unittest.main()