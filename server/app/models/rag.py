from pydantic import BaseModel, Field
from typing import List, Dict, Any

class QueryRequest(BaseModel):
    """Request model for a RAG query."""
    question: str = Field(..., description="The user's question to be answered.")
    top_k: int = Field(5, description="The number of document chunks to retrieve.")
    target_language: str | None = Field(None, description="Optional: Target language for translation (e.g., 'ur' for Urdu).")

class SourceDocument(BaseModel):
    """Represents a source document used for the answer."""
    source: str
    content: str
    url: str | None = None
    
class QueryResponse(BaseModel):
    """Response model for a RAG query."""
    answer: str = Field(..., description="The generated answer.")
    sources: List[SourceDocument] = Field(..., description="A list of source documents that informed the answer.")