from fastapi import APIRouter, Request, HTTPException, Depends
from fastapi.responses import StreamingResponse
from typing import AsyncGenerator
import json
import asyncio
from uuid import UUID

from backend.services.chatbot_service import chatbot_service
from backend.services.embedding_service import embedding_service
from backend.services.db_service import db_service
from backend.models.schemas import ChatRequest, ChatResponse
from backend.config import settings

router = APIRouter()

# Dependency for API Key authentication
async def get_api_key(request: Request):
    api_key = request.headers.get("X-API-Key")
    if not api_key or api_key != settings.FASTAPI_API_KEY:
        raise HTTPException(status_code=401, detail="Unauthorized")
    return api_key

@router.get("/health")
async def health_check():
    return {"status": "ok"}

@router.post("/chat")
async def chat_endpoint(chat_request: ChatRequest, api_key: str = Depends(get_api_key)):
    session_id = chat_request.session_id
    if not session_id:
        session_id = await db_service.create_session() # Create new session if not provided
        await db_service.connect() # Ensure connection is open if not already

    response_generator = chatbot_service.get_rag_response_stream(
        session_id=session_id,
        user_query=chat_request.query,
        highlighted_text=chat_request.highlighted_text
    )

    async def stream_response():
        try:
            async for chunk in response_generator:
                yield f"data: {json.dumps({'content': chunk, 'session_id': session_id})}\n\n"
            yield "data: [DONE]\n\n"
        except Exception as e:
            print(f"Streaming error: {e}")
            yield f"data: {json.dumps({'error': str(e)})}\n\n"
            yield "data: [DONE]\n\n"

    return StreamingResponse(stream_response(), media_type="text/event-stream")

@router.post("/embed")
async def embed_content_endpoint(content_to_embed: List[str], api_key: str = Depends(get_api_key)):
    # This endpoint could be used for on-demand embedding if needed,
    # but the primary ingestion will likely be via the script in backend/scripts/
    try:
        # Assuming upload_vectors can take a list of texts and an optional list of metadatas
        await embedding_service.upload_vectors(content_to_embed)
        return {"status": "success", "message": f"Embedded {len(content_to_embed)} content chunks."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to embed content: {e}")
