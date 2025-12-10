"""
Comprehensive test script to validate all implemented features
"""
import os
import sys
import requests
import json
from typing import Dict, Any, List
import time

# Add the backend src directory to the path to import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

# Configuration
BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
TEST_EMAIL = "test@example.com"
TEST_PASSWORD = "secure_password_123"
TEST_USER_NAME = "Test User"

def test_backend_health():
    """Test if the backend is running and accessible"""
    try:
        response = requests.get(f"{BACKEND_URL}/health")
        if response.status_code == 200:
            print("✓ Backend health check passed")
            return True
        else:
            print(f"✗ Backend health check failed with status {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Backend health check failed with error: {e}")
        return False

def test_signup():
    """Test user signup functionality"""
    try:
        signup_data = {
            "email": TEST_EMAIL,
            "password": TEST_PASSWORD,
            "name": TEST_USER_NAME,
            "technical_skills": ["Python", "ROS", "Machine Learning"],
            "experience_level": "intermediate",
            "background_questionnaire": {
                "software_exp": "5 years of Python development",
                "hardware_access": "Simulation environment",
                "ros_knowledge": "Basic ROS 2 knowledge"
            }
        }
        
        response = requests.post(f"{BACKEND_URL}/api/auth/signup", json=signup_data)
        
        if response.status_code in [200, 201]:
            print("✓ Signup test passed")
            return response.json()
        else:
            print(f"✗ Signup test failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return None
    except Exception as e:
        print(f"✗ Signup test failed with error: {e}")
        return None

def test_login():
    """Test user login functionality"""
    try:
        login_data = {
            "email": TEST_EMAIL,
            "password": TEST_PASSWORD
        }
        
        response = requests.post(f"{BACKEND_URL}/api/auth/login", json=login_data)
        
        if response.status_code in [200, 201]:
            print("✓ Login test passed")
            return response.json()
        else:
            print(f"✗ Login test failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return None
    except Exception as e:
        print(f"✗ Login test failed with error: {e}")
        return None

def test_personalization(user_id: str, chapter_id: int):
    """Test personalization functionality"""
    try:
        data = {
            "chapter_id": chapter_id,
            "user_id": user_id
        }
        
        response = requests.post(f"{BACKEND_URL}/api/personalize", json=data)
        
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Personalization test passed for chapter {chapter_id}")
            print(f"  - Personalized content length: {len(result['content'])} chars")
            print(f"  - User level: {result['user_experience_level']}")
            return result
        else:
            print(f"✗ Personalization test failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return None
    except Exception as e:
        print(f"✗ Personalization test failed with error: {e}")
        return None

def test_urdu_translation(chapter_id: int):
    """Test Urdu translation functionality"""
    try:
        data = {
            "chapter_id": chapter_id
        }
        
        response = requests.post(f"{BACKEND_URL}/api/translate/chapter-urdu", json=data)
        
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Urdu translation test passed for chapter {chapter_id}")
            print(f"  - Translated content length: {len(result['urdu_content'])} chars")
            return result
        else:
            print(f"✗ Urdu translation test failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return None
    except Exception as e:
        print(f"✗ Urdu translation test failed with error: {e}")
        return None

def test_rag_functionality():
    """Test RAG functionality with a sample query"""
    try:
        query_data = {
            "query": "What is ROS 2 and how is it used in robotics?",
            "selected_text": "Robot Operating System"
        }
        
        response = requests.post(f"{BACKEND_URL}/api/chat/query", json=query_data)
        
        if response.status_code == 200:
            result = response.json()
            print("✓ RAG functionality test passed")
            print(f"  - Answer length: {len(result['answer'])} chars")
            print(f"  - Number of sources: {len(result['sources'])}")
            return result
        else:
            print(f"✗ RAG functionality test failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return None
    except Exception as e:
        print(f"✗ RAG functionality test failed with error: {e}")
        return None

def test_module_and_chapter_endpoints():
    """Test getting modules and chapters"""
    try:
        # Test getting modules
        response = requests.get(f"{BACKEND_URL}/api/modules")
        if response.status_code == 200:
            modules = response.json()
            print(f"✓ Modules endpoint test passed - found {len(modules)} modules")
            
            # Test getting chapters for the first module if available
            if modules:
                module_id = modules[0]['id']
                response = requests.get(f"{BACKEND_URL}/api/modules/{module_id}/chapters")
                if response.status_code == 200:
                    chapters = response.json()
                    print(f"✓ Chapters endpoint test passed - found {len(chapters)} chapters in module {module_id}")
                    return modules, chapters
                else:
                    print(f"✗ Chapters endpoint test failed with status {response.status_code}")
                    return modules, None
        else:
            print(f"✗ Modules endpoint test failed with status {response.status_code}")
            return None, None
    except Exception as e:
        print(f"✗ Modules/chapters test failed with error: {e}")
        return None, None

def run_comprehensive_tests():
    """Run all the tests to validate the implemented features"""
    print("Starting comprehensive validation of all features...\n")
    
    all_tests_passed = True
    
    # Test 1: Backend availability
    print("1. Testing backend health...")
    if not test_backend_health():
        all_tests_passed = False
    print()
    
    # Test 2: Authentication functionality
    print("2. Testing authentication functionality...")
    signup_result = test_signup()
    if not signup_result:
        all_tests_passed = False
    else:
        print("  - Signup successful")
    
    login_result = test_login()
    if not login_result:
        all_tests_passed = False
    else:
        print("  - Login successful")
    print()
    
    # Test 3: Get available modules/chapters for other tests
    print("3. Testing modules and chapters endpoints...")
    modules, chapters = test_module_and_chapter_endpoints()
    if not modules or not chapters:
        all_tests_passed = False
        print("  - Skipping personalization and translation tests due to missing content")
    print()
    
    # Test 4: Personalization (only if content exists)
    if chapters:
        print("4. Testing personalization functionality...")
        # Using a sample user ID and the first chapter ID
        first_chapter_id = chapters[0]['id'] if chapters else 1
        personalization_result = test_personalization("test_user_123", first_chapter_id)
        if not personalization_result:
            all_tests_passed = False
        print()
    
    # Test 5: Urdu Translation (only if content exists)
    if chapters:
        print("5. Testing Urdu translation functionality...")
        first_chapter_id = chapters[0]['id'] if chapters else 1
        translation_result = test_urdu_translation(first_chapter_id)
        if not translation_result:
            all_tests_passed = False
        print()
    
    # Test 6: RAG functionality
    print("6. Testing RAG functionality...")
    rag_result = test_rag_functionality()
    if not rag_result:
        all_tests_passed = False
    print()
    
    # Summary
    print("="*50)
    if all_tests_passed:
        print("🎉 All tests passed! All features are working correctly.")
        print("The Physical AI & Humanoid Robotics Textbook is ready for deployment!")
    else:
        print("❌ Some tests failed. Please check the implementation.")
    
    print("="*50)
    return all_tests_passed

if __name__ == "__main__":
    success = run_comprehensive_tests()
    sys.exit(0 if success else 1)