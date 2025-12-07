from fastapi import APIRouter, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from qdrant_client import QdrantClient

from app.db.session import get_db
from app.db.vector_store import get_qdrant_client
from app.models.rag import RagQuery, RagResponse
from app.services.rag_service import RagService

router = APIRouter()

@router.post("/query", response_model=RagResponse)
async def query_rag(
    rag_query: RagQuery,
    db: AsyncSession = Depends(get_db),
    qdrant: QdrantClient = Depends(get_qdrant_client),
):
    """
    Accepts a query and returns a RAG-generated response.
    """
    service = RagService(db, qdrant)
    response = await service.query(rag_query.query, rag_query.top_k)
    return response
