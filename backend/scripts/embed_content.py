import os
import asyncio
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import UnexpectedResponse
from langchain.text_splitter import RecursiveCharacterTextSplitter
from openai import OpenAI
from typing import List, Dict

# Assuming embedding_service and settings from backend/services and backend/config can be adapted or re-imported
# For a standalone script, it's often better to re-initialize dependencies or pass them.
# For simplicity, let's re-initialize here.

load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "..", ".env")) # Load from backend/.env

# Re-define Config and OpenAI client for script context, or import if backend structure allows
class ScriptConfig:
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "your_openai_api_key_here")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "your_qdrant_api_key_here")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "https://your-qdrant-cluster-url.cloud.qdrant.io")

script_settings = ScriptConfig()
openai_client = OpenAI(api_key=script_settings.OPENAI_API_KEY)
qdrant_client = QdrantClient(
    url=script_settings.QDRANT_URL,
    api_key=script_settings.QDRANT_API_KEY,
)
embedding_model = "text-embedding-ada-002" # From T002 decision
collection_name = "book_embeddings"

async def get_embedding(text: str) -> List[float]:
    response = openai_client.embeddings.create(
        input=text,
        model=embedding_model
    )
    return response.data[0].embedding

async def create_qdrant_collection():
    try:
        response = openai_client.embeddings.create(input="test", model=embedding_model)
        vector_size = len(response.data[0].embedding)

        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        print(f"Collection '{collection_name}' recreated with vector size {vector_size}.")
    except UnexpectedResponse as e:
        if "already exists" in str(e):
            print(f"Collection '{collection_name}' already exists.")
        else:
            raise

async def embed_and_upload_content(content_dir: str = "../book/docs"):
    await create_qdrant_collection()

    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        length_function=len,
    )

    all_texts = []
    all_metadatas = []

    for root, _, files in os.walk(content_dir):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                file_path = os.path.join(root, file)
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                
                # Basic markdown parsing to remove frontmatter
                if content.startswith("---"):
                    parts = content.split("---", 2)
                    if len(parts) > 2:
                        content = parts[2]
                
                chunks = text_splitter.split_text(content)
                
                for i, chunk in enumerate(chunks):
                    all_texts.append(chunk)
                    all_metadatas.append({
                        "source": os.path.relpath(file_path, content_dir),
                        "chunk_index": i,
                        "title": file # Placeholder, ideally extract from frontmatter or heading
                    })

    if not all_texts:
        print("No Markdown or MDX files found to embed.")
        return

    points = []
    for i, text_chunk in enumerate(all_texts):
        embedding = await get_embedding(text_chunk)
        points.append(
            models.PointStruct(
                id=i, # Unique ID for each chunk
                vector=embedding,
                payload={"text": text_chunk, **all_metadatas[i]}
            )
        )
    
    qdrant_client.upsert(
        collection_name=collection_name,
        wait=True,
        points=points
    )
    print(f"Successfully uploaded {len(all_texts)} embedded chunks to Qdrant.")

if __name__ == "__main__":
    # Ensure this script is run from the 'backend/' directory or adjust paths
    print("Starting content embedding process...")
    asyncio.run(embed_and_upload_content())
    print("Content embedding process finished.")
