import asyncpg
from typing import List, Optional
from datetime import datetime
import uuid

from backend.config import settings
from backend.models.schemas import LogEntry, ChatMessage # Assuming ChatMessage will be defined later

class DBService:
    def __init__(self):
        self.database_url = settings.DATABASE_URL
        self.pool = None

    async def connect(self):
        if not self.pool:
            self.pool = await asyncpg.create_pool(self.database_url)

    async def disconnect(self):
        if self.pool:
            await self.pool.close()
            self.pool = None

    async def create_tables(self):
        async with self.pool.acquire() as conn:
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    session_id UUID PRIMARY KEY,
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
                );
                CREATE TABLE IF NOT EXISTS chat_messages (
                    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    session_id UUID REFERENCES chat_sessions(session_id),
                    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                    sender VARCHAR(50) NOT NULL, -- 'user' or 'bot'
                    content TEXT NOT NULL,
                    highlighted_text TEXT,
                    is_user_message BOOLEAN NOT NULL
                );
                -- Log entries can be separate or integrated into chat_messages
                -- For now, let's assume chat_messages serves as the log.
            """)

    async def create_session(self) -> str:
        session_id = uuid.uuid4()
        async with self.pool.acquire() as conn:
            await conn.execute("INSERT INTO chat_sessions (session_id) VALUES ($1)", session_id)
        return str(session_id)

    async def add_message(self, session_id: str, sender: str, content: str, is_user_message: bool, highlighted_text: Optional[str] = None):
        session_uuid = uuid.UUID(session_id)
        async with self.pool.acquire() as conn:
            await conn.execute(
                "INSERT INTO chat_messages (session_id, sender, content, is_user_message, highlighted_text) VALUES ($1, $2, $3, $4, $5)",
                session_uuid, sender, content, is_user_message, highlighted_text
            )

    async def get_session_messages(self, session_id: str, limit: int = 10) -> List[dict]:
        session_uuid = uuid.UUID(session_id)
        async with self.pool.acquire() as conn:
            records = await conn.fetch(
                "SELECT sender, content, timestamp, is_user_message, highlighted_text FROM chat_messages WHERE session_id = $1 ORDER BY timestamp DESC LIMIT $2",
                session_uuid, limit
            )
        return [dict(r) for r in records]

# Global instance for easy access
db_service = DBService()
