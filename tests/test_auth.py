import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import json
from .conftest import sample_user_data


def test_signup_success(client, sample_user_data):
    """Test successful user signup"""
    with patch('backend.src.auth.auth.register') as mock_register:
        mock_register.return_value = {
            "user": {
                "id": "test_user_id",
                "email": sample_user_data["email"],
                "name": sample_user_data["name"]
            },
            "session": {"token": "mock_token"}
        }

        response = client.post(
            "/auth/sign-up", 
            json=sample_user_data,
            headers={"Content-Type": "application/json"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "user" in data
        assert data["user"]["email"] == sample_user_data["email"]


def test_signup_missing_fields(client):
    """Test signup with missing required fields"""
    incomplete_data = {
        "email": "test@example.com"
        # Missing password and name
    }

    response = client.post(
        "/auth/sign-up", 
        json=incomplete_data,
        headers={"Content-Type": "application/json"}
    )

    assert response.status_code == 422  # Validation error


def test_signin_success(client, sample_user_data):
    """Test successful user signin"""
    signin_data = {
        "email": sample_user_data["email"],
        "password": sample_user_data["password"]
    }

    with patch('backend.src.auth.auth.sign_in_with_password') as mock_signin:
        mock_signin.return_value = {
            "user": {
                "id": "test_user_id",
                "email": sample_user_data["email"],
                "name": sample_user_data["name"]
            },
            "session": {"token": "mock_token"}
        }

        response = client.post(
            "/auth/sign-in", 
            json=signin_data,
            headers={"Content-Type": "application/json"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "user" in data
        assert data["user"]["email"] == sample_user_data["email"]


def test_signin_invalid_credentials(client):
    """Test signin with invalid credentials"""
    invalid_signin_data = {
        "email": "nonexistent@example.com",
        "password": "wrongpassword"
    }

    with patch('backend.src.auth.auth.sign_in_with_password') as mock_signin:
        mock_signin.side_effect = Exception("Invalid email or password")

        response = client.post(
            "/auth/sign-in", 
            json=invalid_signin_data,
            headers={"Content-Type": "application/json"}
        )

        assert response.status_code == 401  # Unauthorized


def test_signin_missing_fields(client):
    """Test signin with missing required fields"""
    incomplete_data = {
        "email": "test@example.com"
        # Missing password
    }

    response = client.post(
        "/auth/sign-in", 
        json=incomplete_data,
        headers={"Content-Type": "application/json"}
    )

    assert response.status_code == 422  # Validation error


def test_me_endpoint_authenticated(client):
    """Test /auth/me endpoint with authenticated user"""
    # Mock the auth middleware to return a user
    mock_user = {
        "id": "test_user_id",
        "email": "test@example.com",
        "name": "Test User"
    }

    with patch('backend.src.auth.auth.user') as mock_auth:
        mock_auth.return_value = mock_user

        response = client.get("/auth/me")

        assert response.status_code == 200
        data = response.json()
        assert data["id"] == mock_user["id"]
        assert data["email"] == mock_user["email"]


def test_me_endpoint_unauthenticated(client):
    """Test /auth/me endpoint without authentication"""
    # Mock the auth middleware to return None (unauthenticated)
    with patch('backend.src.auth.auth.user') as mock_auth:
        mock_auth.return_value = None

        response = client.get("/auth/me")

        assert response.status_code == 401  # Unauthorized


def test_signup_email_validation(client):
    """Test signup with invalid email format"""
    invalid_data = {
        "email": "invalid-email",
        "password": "securepassword123",
        "name": "Test User"
    }

    response = client.post(
        "/auth/sign-up", 
        json=invalid_data,
        headers={"Content-Type": "application/json"}
    )

    assert response.status_code == 422  # Validation error due to invalid email


def test_password_strength_validation(client):
    """Test signup with weak password"""
    weak_password_data = {
        "email": "test@example.com",
        "password": "123",  # Too short
        "name": "Test User"
    }

    response = client.post(
        "/auth/sign-up", 
        json=weak_password_data,
        headers={"Content-Type": "application/json"}
    )

    assert response.status_code == 422  # Validation error due to weak password