from openai import OpenAI
from qdrant_client import QdrantClient, models
from qdrant_client.http.exceptions import UnexpectedResponse
from typing import List, Dict, Optional

from backend.config import settings

class EmbeddingService:
    def __init__(self):
        self.openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
        )
        self.embedding_model = "text-embedding-ada-002" # From T002 decision
        self.collection_name = "book_embeddings"

    async def _get_embedding(self, text: str) -> List[float]:
        response = self.openai_client.embeddings.create(
            input=text,
            model=self.embedding_model
        )
        return response.data[0].embedding

    async def create_collection(self):
        try:
            # Get model metadata to know embedding dimension
            response = self.openai_client.embeddings.create(input="test", model=self.embedding_model)
            vector_size = len(response.data[0].embedding)

            self.qdrant_client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
            )
            print(f"Collection '{self.collection_name}' recreated with vector size {vector_size}.")
        except UnexpectedResponse as e:
            if "already exists" in str(e):
                print(f"Collection '{self.collection_name}' already exists.")
            else:
                raise

    async def upload_vectors(self, texts: List[str], metadatas: Optional[List[Dict]] = None):
        if metadatas is None:
            metadatas = [{} for _ in texts]
        
        points = []
        for i, text in enumerate(texts):
            embedding = await self._get_embedding(text)
            points.append(
                models.PointStruct(
                    id=i, # Unique ID for each chunk
                    vector=embedding,
                    payload={"text": text, **metadatas[i]}
                )
            )
        
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )
        print(f"Uploaded {len(texts)} vectors to '{self.collection_name}'.")

    async def search_vectors(self, query_text: str, limit: int = 3) -> List[Dict]:
        query_embedding = await self._get_embedding(query_text)
        
        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True
        )
        return [{"text": hit.payload["text"], "score": hit.score} for hit in search_result]

embedding_service = EmbeddingService()
