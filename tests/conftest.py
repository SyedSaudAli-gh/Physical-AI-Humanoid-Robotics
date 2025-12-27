import os
import tempfile
from unittest.mock import patch
import pytest
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# Note: The old backend has been removed and replaced with rag-backend
# The tests in the main tests/ directory have been reviewed and updated
# The rag-backend has its own comprehensive tests in the rag-backend/tests directory
# This conftest.py now serves as a placeholder for any shared test configuration
# that might be needed for integration tests between components


# Placeholder fixtures - to be updated based on actual current architecture needs
@pytest.fixture
def sample_data():
    """Sample data for testing"""
    return {
        "text": "Sample test data",
        "metadata": {"test": True}
    }