from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from uuid import UUID

class ChatMessage(BaseModel):
    message_id: UUID
    session_id: UUID
    timestamp: datetime
    sender: str # 'user' or 'bot'
    content: str
    is_user_message: bool
    highlighted_text: Optional[str] = None

class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    highlighted_text: Optional[str] = None

class ChatResponse(BaseModel):
    session_id: str
    response: str
    is_streaming: bool = False
    # You might want to add a field for sources/references from the RAG
    # sources: List[str] = []

class LogEntry(BaseModel): # LogEntry will represent general events, ChatMessage for conversational turns
    session_id: Optional[str] = None # Optional for general logs
    timestamp: datetime = datetime.now() # Default to now for easier logging
    event_type: str # E.g., "user_query", "bot_response", "error"
    details: dict # Flexible dictionary for other log details
