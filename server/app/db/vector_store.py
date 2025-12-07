from qdrant_client import QdrantClient
from app.core.config import settings

def get_qdrant_client():
    """
    Returns a QdrantClient instance.
    """
    client = QdrantClient(
        host=settings.qdrant_host, 
        api_key=settings.qdrant_api_key
    )
    return client

# You can also initialize the collection here if it doesn't exist
# client = get_qdrant_client()
# try:
#     client.get_collection(collection_name=settings.qdrant_collection_name)
# except Exception:
#     client.recreate_collection(
#         collection_name=settings.qdrant_collection_name,
#         vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
#     )
