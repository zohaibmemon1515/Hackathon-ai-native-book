import os
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

class Config:
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "your_openai_api_key_here")
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql://user:password@host:port/database")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "your_qdrant_api_key_here")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "https://your-qdrant-cluster-url.cloud.qdrant.io")
    FASTAPI_API_KEY: str = os.getenv("FASTAPI_API_KEY", "your_fastapi_api_key_here")

# Instantiate the configuration
settings = Config()
