import logging
from fastapi import APIRouter, HTTPException
from uuid import UUID
from src.services.agent_service import agent_service
from src.services.chat_history_service import chat_history_service
from src.models.schemas import ChatRequest, ChatResponse, ChatMessageSchema

logger = logging.getLogger(__name__)
router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        user_id = "test_user"

        # -----------------------------
        # Retrieve or create session
        # -----------------------------
        session = None
        if request.session_id:
            try:
                session_uuid = UUID(request.session_id)
                session = await chat_history_service.get_session_by_id(session_uuid)
                if not session:
                    session = await chat_history_service.create_session(user_id)
            except ValueError:
                session = await chat_history_service.create_session(user_id)
        else:
            session = await chat_history_service.create_session(user_id)

        if not session:
            raise HTTPException(status_code=500, detail="Failed to create or retrieve chat session.")

        # -----------------------------
        # Save user message
        # -----------------------------
        await chat_history_service.save_message(
            session_id=session.id,
            role="user",
            content=request.message
        )

        # -----------------------------
        # Get AI response
        # -----------------------------
        response_content = await agent_service.get_response(
            user_message=request.message
        )

        # -----------------------------
        # Save assistant message
        # -----------------------------
        await chat_history_service.save_message(
            session_id=session.id,
            role="assistant",
            content=response_content
        )

        # -----------------------------
        # Return full history
        # -----------------------------
        full_history_orm = await chat_history_service.get_history(session.id)

        # ✅ Convert UUID to string for Pydantic
        full_history_schema = [
            ChatMessageSchema.model_validate({
                "id": str(msg.id),
                "session_id": str(msg.session_id),
                "role": msg.role,
                "content": msg.content,
                "context_text": None,
                "metadata_": msg.metadata_,
                "created_at": msg.created_at,
            })
            for msg in full_history_orm
        ]

        return ChatResponse(
            session_id=str(session.id),
            response=response_content,
            history=full_history_schema
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))
