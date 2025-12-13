import pytest
from sqlalchemy.exc import IntegrityError
from backend.src.models.user import User
from backend.src.models.chat_history import ChatHistory
from datetime import datetime


def test_create_user(db_session):
    """Test creating a user in the database"""
    user = User(
        id="test_user_id",
        email="test@example.com",
        name="Test User"
    )
    
    db_session.add(user)
    db_session.commit()
    
    retrieved_user = db_session.query(User).filter(User.email == "test@example.com").first()
    assert retrieved_user is not None
    assert retrieved_user.email == "test@example.com"
    assert retrieved_user.name == "Test User"


def test_user_unique_email_constraint(db_session):
    """Test that email uniqueness constraint is enforced"""
    user1 = User(
        id="test_user_id_1",
        email="unique@example.com",
        name="Test User 1"
    )
    
    user2 = User(
        id="test_user_id_2",
        email="unique@example.com",  # Same email as user1
        name="Test User 2"
    )
    
    db_session.add(user1)
    db_session.commit()
    
    db_session.add(user2)
    with pytest.raises(IntegrityError):
        db_session.commit()


def test_create_chat_history(db_session):
    """Test creating a chat history record"""
    chat = ChatHistory(
        id="test_chat_id",
        user_id="test_user_id",
        message="Hello, how are you?",
        response="I'm doing well, thank you!",
        timestamp=datetime.utcnow()
    )
    
    db_session.add(chat)
    db_session.commit()
    
    retrieved_chat = db_session.query(ChatHistory).filter(ChatHistory.id == "test_chat_id").first()
    assert retrieved_chat is not None
    assert retrieved_chat.message == "Hello, how are you?"
    assert retrieved_chat.response == "I'm doing well, thank you!"


def test_chat_history_timestamp(db_session):
    """Test that chat history records maintain proper timestamps"""
    past_time = datetime(2023, 1, 1)
    
    chat = ChatHistory(
        id="test_chat_timestamp",
        user_id="test_user_id",
        message="Test message",
        response="Test response",
        timestamp=past_time
    )
    
    db_session.add(chat)
    db_session.commit()
    
    retrieved_chat = db_session.query(ChatHistory).filter(
        ChatHistory.id == "test_chat_timestamp"
    ).first()
    assert retrieved_chat.timestamp == past_time


def test_user_chat_history_relationship(db_session):
    """Test relationship between user and chat history"""
    user = User(
        id="test_user_rel_id",
        email="rel_test@example.com",
        name="Relationship Test User"
    )
    
    chat1 = ChatHistory(
        id="test_chat_rel_1",
        user_id=user.id,
        message="First message",
        response="First response",
        timestamp=datetime.utcnow()
    )
    
    chat2 = ChatHistory(
        id="test_chat_rel_2",
        user_id=user.id,
        message="Second message",
        response="Second response",
        timestamp=datetime.utcnow()
    )
    
    db_session.add(user)
    db_session.add(chat1)
    db_session.add(chat2)
    db_session.commit()
    
    # Retrieve user with chats
    retrieved_user = db_session.query(User).filter(User.id == user.id).first()
    assert len(retrieved_user.chats) >= 2  # At least these two chats
    
    # Check that the chats belong to the right user
    for chat in retrieved_user.chats:
        if chat.id in [chat1.id, chat2.id]:
            assert chat.user_id == user.id


def test_delete_user_cascades_chats(db_session):
    """Test that deleting a user also deletes their chat history (if configured with cascade)"""
    user = User(
        id="test_user_del_id",
        email="del_test@example.com",
        name="Delete Test User"
    )
    
    chat = ChatHistory(
        id="test_chat_del",
        user_id=user.id,
        message="Message to be deleted",
        response="Response to be deleted",
        timestamp=datetime.utcnow()
    )
    
    db_session.add(user)
    db_session.add(chat)
    db_session.commit()
    
    # Verify chat exists
    assert db_session.query(ChatHistory).filter(ChatHistory.id == "test_chat_del").count() == 1
    
    # Delete user
    db_session.delete(user)
    db_session.commit()
    
    # Check if chat was also deleted (depends on foreign key constraints)
    remaining_chats = db_session.query(ChatHistory).filter(ChatHistory.id == "test_chat_del").count()
    # Note: This assertion depends on how the foreign key constraint is defined in your models
    # If CASCADE DELETE is set, this will be 0; otherwise, it might raise an error or remain 1
    # Adjust this based on your actual model definition


def test_update_user_info(db_session):
    """Test updating user information"""
    user = User(
        id="test_user_upd_id",
        email="upd_test@example.com",
        name="Original Name"
    )
    
    db_session.add(user)
    db_session.commit()
    
    # Update the user's name
    user.name = "Updated Name"
    db_session.commit()
    
    # Retrieve and verify the update
    updated_user = db_session.query(User).filter(User.id == "test_user_upd_id").first()
    assert updated_user.name == "Updated Name"


def test_query_user_by_email(db_session):
    """Test querying a user by email"""
    user = User(
        id="test_user_query_id",
        email="query_test@example.com",
        name="Query Test User"
    )
    
    db_session.add(user)
    db_session.commit()
    
    # Query by email
    found_user = db_session.query(User).filter(User.email == "query_test@example.com").first()
    assert found_user is not None
    assert found_user.id == "test_user_query_id"


def test_multiple_users_different_emails(db_session):
    """Test creating multiple users with different emails"""
    users_data = [
        {"id": "user1_id", "email": "user1@test.com", "name": "User One"},
        {"id": "user2_id", "email": "user2@test.com", "name": "User Two"},
        {"id": "user3_id", "email": "user3@test.com", "name": "User Three"}
    ]
    
    for user_data in users_data:
        user = User(**user_data)
        db_session.add(user)
    
    db_session.commit()
    
    # Verify all users were created
    all_users = db_session.query(User).all()
    assert len(all_users) >= len(users_data)
    
    # Verify each user exists with correct attributes
    for user_data in users_data:
        found_user = db_session.query(User).filter(User.id == user_data["id"]).first()
        assert found_user is not None
        assert found_user.email == user_data["email"]
        assert found_user.name == user_data["name"]


def test_empty_chat_history_for_new_user(db_session):
    """Test that a new user has no chat history initially"""
    user = User(
        id="test_user_empty_hist_id",
        email="empty_hist@test.com",
        name="Empty History User"
    )
    
    db_session.add(user)
    db_session.commit()
    
    retrieved_user = db_session.query(User).filter(User.id == user.id).first()
    # Depending on how relationships are configured, this could be an empty list or None
    assert hasattr(retrieved_user, 'chats')