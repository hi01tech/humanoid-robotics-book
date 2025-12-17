# import os
# from pydantic_settings import BaseSettings
# from dotenv import load_dotenv

# # Load .env file from the root directory
# dotenv_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '.env')
# load_dotenv(dotenv_path=dotenv_path)

# class Settings(BaseSettings):
#     # Neon Postgres
#     database_url: str = os.getenv("DATABASE_URL")

#     # Qdrant
#     qdrant_host: str = os.getenv("QDRANT_HOST")
#     qdrant_api_key: str = os.getenv("QDRANT_API_KEY")
#     qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "textbook_rag")

#     # Google AI
#     google_api_key: str = os.getenv("GOOGLE_API_KEY")


#     class Config:
#         case_sensitive = True
        
# settings = Settings()


import os
from pathlib import Path
from pydantic_settings import BaseSettings
from dotenv import load_dotenv

# 1. Get the path to the 'server' folder
# Path(__file__) is config.py
# .parent is 'core'
# .parent.parent is 'app'
# .parent.parent.parent is 'server'
server_dir = Path(__file__).resolve().parent.parent.parent
dotenv_path = server_dir / ".env"

# 2. Load that specific file
load_dotenv(dotenv_path=dotenv_path)

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

# This helps you verify it's working
if not settings.database_url:
    print(f" ERROR: .env file not found or empty at: {dotenv_path}")
else:
    print(f" SUCCESS: Loaded config from: {dotenv_path}")