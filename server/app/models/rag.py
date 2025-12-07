from pydantic import BaseModel
from typing import List, Optional

class RagQuery(BaseModel):
    query: str
    top_k: int = 5

class RagResponse(BaseModel):
    query: str
    retrieved_chunks: List[str]
    response: str
