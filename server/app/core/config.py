import os
from pydantic import BaseSettings
from dotenv import load_dotenv

load_dotenv()

class Settings(BaseSettings):
    # Neon Postgres
    database_url: str = os.getenv("DATABASE_URL")

    # Qdrant
    qdrant_host: str = os.getenv("QDRANT_HOST")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "textbook_rag")

    # Google AI
    google_api_key: str = os.getenv("GOOGLE_API_KEY")

    class Config:
        case_sensitive = True

settings = Settings()