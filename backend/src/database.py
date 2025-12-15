from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from .config import settings
import logging
from typing import Dict, List, Any
from datetime import datetime
import uuid

logger = logging.getLogger(__name__)

# Try to create SQLAlchemy engine
DB_AVAILABLE = False
try:
    engine = create_engine(
        settings.database_url,
        pool_pre_ping=True,  # Verify connections before using
        echo=settings.debug,  # Log SQL queries in debug mode
    )
    # Test connection
    with engine.connect() as conn:
        conn.execute("SELECT 1")
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    DB_AVAILABLE = True
    logger.info("Database connection established")
except Exception as e:
    logger.warning(f"Database unavailable, using in-memory storage: {e}")
    SessionLocal = None

# Base class for models
Base = declarative_base()


# In-memory storage for when database is unavailable
class InMemoryStore:
    """Simple in-memory storage for conversations and messages."""

    def __init__(self):
        self.conversations: Dict[str, Dict[str, Any]] = {}
        self.messages: Dict[str, List[Dict[str, Any]]] = {}

    def create_conversation(self, conversation_id: str, title: str = None, language: str = "en"):
        """Create a new conversation."""
        self.conversations[conversation_id] = {
            "id": conversation_id,
            "title": title,
            "language": language,
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
        }
        self.messages[conversation_id] = []
        return self.conversations[conversation_id]

    def get_conversation(self, conversation_id: str):
        """Get a conversation by ID."""
        return self.conversations.get(conversation_id)

    def add_message(self, conversation_id: str, message: Dict[str, Any]):
        """Add a message to a conversation."""
        if conversation_id not in self.messages:
            self.messages[conversation_id] = []
        self.messages[conversation_id].append(message)
        # Update conversation timestamp
        if conversation_id in self.conversations:
            self.conversations[conversation_id]["updated_at"] = datetime.utcnow()

    def get_messages(self, conversation_id: str, limit: int = None):
        """Get messages for a conversation."""
        messages = self.messages.get(conversation_id, [])
        if limit:
            return messages[-limit:]
        return messages

    def update_conversation_title(self, conversation_id: str, title: str):
        """Update conversation title."""
        if conversation_id in self.conversations:
            self.conversations[conversation_id]["title"] = title


# Global in-memory store instance
memory_store = InMemoryStore()


# Dependency for FastAPI routes
def get_db():
    """Get database session or in-memory store."""
    if DB_AVAILABLE:
        db = SessionLocal()
        try:
            yield db
        finally:
            db.close()
    else:
        # Return in-memory store
        yield memory_store
