from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any, Union
from datetime import datetime
import uuid

from ..database import get_db, InMemoryStore, DB_AVAILABLE
from ..models.conversation import Conversation, Message, MessageRole
from ..services.rag_service import rag_service
import logging

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/chat", tags=["chat"])


# Pydantic schemas for request/response
class CreateConversationRequest(BaseModel):
    title: Optional[str] = None
    language: str = "en"


class CreateConversationResponse(BaseModel):
    conversation_id: str
    created_at: str


class SendMessageRequest(BaseModel):
    content: str = Field(..., min_length=1, max_length=10000)
    selected_context: Optional[Dict[str, str]] = None


class MessageResponse(BaseModel):
    id: str
    role: str
    content: str
    citations: Optional[List[Dict[str, Any]]] = None
    created_at: str


class SendMessageResponse(BaseModel):
    user_message: MessageResponse
    assistant_message: MessageResponse


class ConversationListItem(BaseModel):
    id: str
    title: Optional[str]
    created_at: str
    updated_at: str
    message_count: int


@router.post("/conversations", response_model=CreateConversationResponse)
async def create_conversation(
    request: CreateConversationRequest,
    db: Union[Session, InMemoryStore] = Depends(get_db)
):
    """
    Create a new conversation.

    - For anonymous users, user_id will be null
    - Title is auto-generated from first message if not provided
    """
    try:
        conversation_id = str(uuid.uuid4())

        if isinstance(db, InMemoryStore):
            # Use in-memory storage
            conversation = db.create_conversation(
                conversation_id=conversation_id,
                title=request.title,
                language=request.language
            )
            logger.info(f"Created conversation {conversation_id} (in-memory)")
            return CreateConversationResponse(
                conversation_id=conversation_id,
                created_at=conversation["created_at"].isoformat()
            )
        else:
            # Use database
            conversation = Conversation(
                id=uuid.UUID(conversation_id),
                user_id=None,  # Will be set when auth is implemented
                title=request.title,
                language=request.language
            )

            db.add(conversation)
            db.commit()
            db.refresh(conversation)

            logger.info(f"Created conversation {conversation.id}")

            return CreateConversationResponse(
                conversation_id=str(conversation.id),
                created_at=conversation.created_at.isoformat()
            )

    except Exception as e:
        logger.error(f"Error creating conversation: {e}")
        if not isinstance(db, InMemoryStore):
            db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create conversation"
        )


@router.post("/conversations/{conversation_id}/messages", response_model=SendMessageResponse)
async def send_message(
    conversation_id: str,
    request: SendMessageRequest,
    db: Union[Session, InMemoryStore] = Depends(get_db)
):
    """
    Send a message and get AI response.

    - Creates user message
    - Searches book content
    - Generates AI response with citations
    - Stores both messages
    """
    try:
        if isinstance(db, InMemoryStore):
            # Use in-memory storage
            conversation = db.get_conversation(conversation_id)

            if not conversation:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail="Conversation not found"
                )

            # Get conversation history
            prev_messages = db.get_messages(conversation_id, limit=10)
            conversation_history = [
                {"role": msg["role"], "content": msg["content"]}
                for msg in prev_messages
            ]

            # Extract selected context
            selected_text = None
            if request.selected_context:
                selected_text = request.selected_context.get("text")

            # Use RAG service to get response
            result = rag_service.ask_question(
                query=request.content,
                conversation_history=conversation_history,
                selected_context=selected_text
            )

            # Create user message
            user_msg_id = str(uuid.uuid4())
            user_message = {
                "id": user_msg_id,
                "role": "user",
                "content": request.content,
                "citations": None,
                "created_at": datetime.utcnow().isoformat()
            }
            db.add_message(conversation_id, user_message)

            # Create assistant message
            assistant_msg_id = str(uuid.uuid4())
            assistant_message = {
                "id": assistant_msg_id,
                "role": "assistant",
                "content": result["response"],
                "citations": result["citations"],
                "created_at": datetime.utcnow().isoformat()
            }
            db.add_message(conversation_id, assistant_message)

            # Update conversation title if first message
            if not conversation["title"] and len(prev_messages) == 0:
                db.update_conversation_title(conversation_id, request.content[:50])

            logger.info(f"Sent message in conversation {conversation_id} (in-memory)")

            return SendMessageResponse(
                user_message=MessageResponse(**user_message),
                assistant_message=MessageResponse(**assistant_message)
            )
        else:
            # Use database
            # Verify conversation exists
            conversation = db.query(Conversation).filter(
                Conversation.id == uuid.UUID(conversation_id)
            ).first()

            if not conversation:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail="Conversation not found"
                )

            # Get conversation history
            prev_messages = db.query(Message).filter(
                Message.conversation_id == uuid.UUID(conversation_id)
            ).order_by(Message.created_at.desc()).limit(10).all()

            conversation_history = [
                {"role": msg.role.value, "content": msg.content}
                for msg in reversed(prev_messages)
            ]

            # Extract selected context
            selected_text = None
            if request.selected_context:
                selected_text = request.selected_context.get("text")

            # Use RAG service to get response
            result = rag_service.ask_question(
                query=request.content,
                conversation_history=conversation_history,
                selected_context=selected_text
            )

            # Create user message
            user_message = Message(
                id=uuid.uuid4(),
                conversation_id=uuid.UUID(conversation_id),
                role=MessageRole.USER,
                content=request.content,
                citations=None
            )
            db.add(user_message)

            # Create assistant message
            assistant_message = Message(
                id=uuid.uuid4(),
                conversation_id=uuid.UUID(conversation_id),
                role=MessageRole.ASSISTANT,
                content=result["response"],
                citations=result["citations"]
            )
            db.add(assistant_message)

            # Update conversation title if first message
            if not conversation.title and len(prev_messages) == 0:
                conversation.title = request.content[:50]

            # Update conversation timestamp
            conversation.updated_at = datetime.utcnow()

            db.commit()
            db.refresh(user_message)
            db.refresh(assistant_message)

            logger.info(f"Sent message in conversation {conversation_id}")

            return SendMessageResponse(
                user_message=MessageResponse(**user_message.to_dict()),
                assistant_message=MessageResponse(**assistant_message.to_dict())
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error sending message: {e}")
        if not isinstance(db, InMemoryStore):
            db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=str(e)
        )


@router.get("/conversations/{conversation_id}/messages", response_model=List[MessageResponse])
async def get_messages(
    conversation_id: str,
    db: Union[Session, InMemoryStore] = Depends(get_db)
):
    """Get all messages in a conversation."""
    try:
        if isinstance(db, InMemoryStore):
            # Use in-memory storage
            messages = db.get_messages(conversation_id)
            return [MessageResponse(**msg) for msg in messages]
        else:
            # Use database
            messages = db.query(Message).filter(
                Message.conversation_id == uuid.UUID(conversation_id)
            ).order_by(Message.created_at.asc()).all()

            return [MessageResponse(**msg.to_dict()) for msg in messages]

    except Exception as e:
        logger.error(f"Error fetching messages: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to fetch messages"
        )


@router.get("/conversations", response_model=List[ConversationListItem])
async def list_conversations(
    db: Session = Depends(get_db)
):
    """
    List all conversations.
    
    - For anonymous users, returns empty array
    - For authenticated users, returns their conversations (to be implemented)
    """
    try:
        # For now, return empty for anonymous users
        # Will be filtered by user_id when auth is implemented
        return []
        
    except Exception as e:
        logger.error(f"Error listing conversations: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to list conversations"
        )
