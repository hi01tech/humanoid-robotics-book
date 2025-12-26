from fastapi import APIRouter, Depends
from qdrant_client import AsyncQdrantClient
from ..db.vector_store import get_qdrant_client
from ..models.rag import QueryRequest, QueryResponse # Updated import
from ..services.rag_service import RagService

router = APIRouter()

@router.post("/query", response_model=QueryResponse) # Updated response_model
async def query_rag(
    rag_query: QueryRequest, # Updated request model
    qdrant: AsyncQdrantClient = Depends(get_qdrant_client),
):
    """
    Accepts a question and returns a RAG-generated response.
    """
    service = RagService(qdrant)
    response = await service.query(rag_query.question, rag_query.top_k) # Accessing rag_query.question
    return response