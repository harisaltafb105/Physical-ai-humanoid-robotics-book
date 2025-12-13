from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid

from ..database import get_db
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
    db: Session = Depends(get_db)
):
    """
    Create a new conversation.
    
    - For anonymous users, user_id will be null
    - Title is auto-generated from first message if not provided
    """
    try:
        conversation = Conversation(
            id=uuid.uuid4(),
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
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create conversation"
        )


@router.post("/conversations/{conversation_id}/messages", response_model=SendMessageResponse)
async def send_message(
    conversation_id: str,
    request: SendMessageRequest,
    db: Session = Depends(get_db)
):
    """
    Send a message and get AI response.
    
    - Creates user message
    - Searches book content
    - Generates AI response with citations
    - Stores both messages
    """
    try:
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
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=str(e)
        )


@router.get("/conversations/{conversation_id}/messages", response_model=List[MessageResponse])
async def get_messages(
    conversation_id: str,
    db: Session = Depends(get_db)
):
    """Get all messages in a conversation."""
    try:
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
