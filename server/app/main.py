from fastapi import FastAPI
from app.api import rag

app = FastAPI(
    title="Humanoid Robotics Textbook RAG API",
    description="An API for interacting with the RAG system for the textbook.",
    version="0.1.0",
)

app.include_router(rag.router, prefix="/rag", tags=["RAG"])

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG API"}
