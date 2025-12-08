from fastapi import APIRouter, Depends, HTTPException
from app.models.rag import QueryRequest, QueryResponse
from app.services.rag_service import get_rag_answer

router = APIRouter()

@router.post("/query", response_model=QueryResponse)
async def query_rag(request: QueryRequest):
    """
    Accepts a user's question, retrieves relevant context from the vector store,
    and generates a final answer using a Large Language Model.
    """
    try:
        response = await get_rag_answer(request.question, request.top_k)
        return response
    except Exception as e:
        # For a production app, you'd want more specific error handling
        # and logging.
        print(f"An error occurred during query processing: {e}")
        raise HTTPException(status_code=500, detail="Failed to process the RAG query.")