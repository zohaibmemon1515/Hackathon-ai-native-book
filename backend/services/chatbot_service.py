from typing import List, Dict, AsyncGenerator, Optional
from backend.services.embedding_service import embedding_service
from backend.services.db_service import db_service
from backend.models.schemas import ChatMessage, LogEntry
from backend.config import settings
import google.generativeai as genai
import asyncio

class ChatbotService:
    def __init__(self):
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model_name = "gemini-2.5flash"  # Or: gemini-2.0-pro, gemini-pro

    async def _generate_response(self, messages: List[Dict]) -> AsyncGenerator[str, None]:
        """
        Streaming wrapper around Gemini synchronous streaming.
        """
        try:
            # Convert messages to Gemini format
            full_prompt = ""
            for msg in messages:
                role = msg["role"]
                if role == "system":
                    full_prompt += f"[SYSTEM]\n{msg['content']}\n\n"
                elif role == "assistant":
                    full_prompt += f"[ASSISTANT]\n{msg['content']}\n\n"
                else:
                    full_prompt += f"[USER]\n{msg['content']}\n\n"

            # Run synchronous Gemini stream inside a thread
            stream = await asyncio.to_thread(
                lambda: genai.GenerativeModel(self.model_name).generate_content(
                    full_prompt,
                    stream=True
                )
            )

            # Stream output tokens
            for chunk in stream:
                if chunk.text:
                    yield chunk.text

        except Exception as e:
            print("Gemini Error:", e)
            yield "Sorry, I'm having trouble connecting to Gemini right now."

    async def get_rag_response_stream(
        self, 
        session_id: str, 
        user_query: str, 
        highlighted_text: Optional[str] = None
    ) -> AsyncGenerator[str, None]:

        # 1. Retrieve chat history
        history = await db_service.get_session_messages(session_id, limit=5)
        chat_history_messages = [
            {"role": "user" if msg['is_user_message'] else "assistant", "content": msg['content']}
            for msg in history
        ]
        chat_history_messages.reverse()

        # Store user message
        await db_service.add_message(session_id, "user", user_query, True, highlighted_text)

        # 2. RAG — relevant chunks
        rag_query = f"User query: {user_query}"
        if highlighted_text:
            rag_query = f"User query based on highlighted text: '{highlighted_text}'. Question: {user_query}"

        relevant_chunks = await embedding_service.search_vectors(rag_query, limit=5)
        context = "\n".join([chunk["text"] for chunk in relevant_chunks])

        if not context:
            apology = (
                "I can only answer questions based on the book's content, "
                "and I couldn’t find relevant information for your query."
            )
            await db_service.add_message(session_id, "bot", apology, False)
            yield apology
            return

        # 3. Build prompt
        system = {
            "role": "system",
            "content": (
                "You are a RAG-based AI assistant. Answer ONLY from the book content. "
                "If not present, say you cannot answer from the provided material."
            )
        }
        user_msg = {
            "role": "user",
            "content": f"Book Content:\n{context}\n\nUser Question: {user_query}"
        }

        messages = [system] + chat_history_messages + [user_msg]

        # 4. Stream Gemini response
        full_response = ""
        async for chunk in self._generate_response(messages):
            full_response += chunk
            yield chunk

        # 5. Save response
        await db_service.add_message(session_id, "bot", full_response, False)


chatbot_service = ChatbotService()
