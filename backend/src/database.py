from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from .config import settings
import logging
from typing import Dict, List, Any
from datetime import datetime
import uuid

logger = logging.getLogger(__name__)

# Try to create async SQLAlchemy engine
DB_AVAILABLE = False
engine = None
SessionLocal = None

try:
    engine = create_async_engine(
        settings.database_url,
        pool_pre_ping=True,
        echo=settings.debug,
        pool_size=5,
        max_overflow=10,
    )
    SessionLocal = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)
    DB_AVAILABLE = True
    logger.info("Async database connection configured")
except Exception as e:
    logger.warning(f"Database unavailable, using in-memory storage: {e}")
    engine = None
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
async def get_db():
    """Get async database session or in-memory store."""
    if DB_AVAILABLE and SessionLocal:
        async with SessionLocal() as session:
            try:
                yield session
            except Exception as e:
                await session.rollback()
                raise
    else:
        # Return in-memory store
        yield memory_store
