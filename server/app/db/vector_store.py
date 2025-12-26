from qdrant_client import AsyncQdrantClient
from ..core.config import settings

def get_qdrant_client() -> AsyncQdrantClient:
    """
    Creates and returns a synchronous Qdrant client.
    """
    # Initialize the client with the URL and API Key from settings
    client = AsyncQdrantClient(
        url=settings.qdrant_host,
        api_key=settings.qdrant_api_key,
    )
    return client