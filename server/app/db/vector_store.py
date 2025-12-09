from qdrant_client import QdrantClient
from app.core.config import settings

def get_qdrant_client() -> QdrantClient:
    """
    Creates and returns a synchronous Qdrant client.
    """
    # Initialize the client with the URL and API Key from settings
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )
    return client