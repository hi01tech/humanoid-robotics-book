import os
import uuid
from pathlib import Path
from typing import List, Dict, Any

from langchain_core.documents import Document
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import UnstructuredMarkdownLoader
from sentence_transformers import SentenceTransformer
from qdrant_client import models

from app.db.vector_store import get_qdrant_client
from app.core.config import settings

# --- Configuration ---
# Two levels up from services/ to the project root
DOCS_PATH = Path(__file__).parent.parent.parent.parent / "docs"
EMBEDDING_MODEL = 'all-MiniLM-L6-v2'
CHUNK_SIZE = 1000
CHUNK_OVERLAP = 200


def load_markdown_documents() -> List[Document]:
    """Recursively loads all markdown files from the docs directory."""
    print(f"Loading markdown files from: {DOCS_PATH.resolve()}")
    md_files = list(DOCS_PATH.rglob("*.md"))
    print(f"Found {len(md_files)} markdown files.")
    
    documents = []
    for file_path in md_files:
        print(f"  - Loading: {file_path.name}")
        loader = UnstructuredMarkdownLoader(str(file_path))
        docs = loader.load()
        # Add the source file path to the metadata
        for doc in docs:
            doc.metadata['source'] = str(file_path.relative_to(DOCS_PATH))
        documents.extend(docs)
        
    print(f"Total documents loaded: {len(documents)}")
    return documents

def chunk_documents(documents: List[Document]) -> List[Document]:
    """Splits documents into smaller chunks."""
    print("Chunking documents...")
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=CHUNK_SIZE,
        chunk_overlap=CHUNK_OVERLAP,
        separators=["\n\n", "\n", " ", ""],
    )
    chunked_docs = text_splitter.split_documents(documents)
    print(f"Created {len(chunked_docs)} chunks.")
    return chunked_docs

def generate_embeddings(chunks: List[Document], model: SentenceTransformer) -> List[List[float]]:
    """Generates vector embeddings for a list of document chunks."""
    print("Generating embeddings...")
    contents = [chunk.page_content for chunk in chunks]
    embeddings = model.encode(contents, show_progress_bar=True)
    print("Embeddings generated.")
    return embeddings.tolist()

def upsert_to_qdrant(chunks: List[Document], embeddings: List[List[float]]):
    """Upserts document chunks and their embeddings into Qdrant."""
    print("Upserting vectors to Qdrant...")
    qdrant_client = get_qdrant_client()
    collection_name = settings.qdrant_collection_name
    
    # Ensure the collection exists
    try:
        qdrant_client.get_collection(collection_name=collection_name)
        print(f"Collection '{collection_name}' already exists.")
    except Exception:
        print(f"Collection '{collection_name}' not found. Creating new collection.")
        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=len(embeddings[0]), # Vector size
                distance=models.Distance.COSINE
            ),
        )
        print("Collection created.")

    # Prepare points for upsert
    points = [
        models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "text": chunk.page_content,
                "metadata": chunk.metadata
            }
        )
        for chunk, embedding in zip(chunks, embeddings)
    ]
    
    # Upsert in batches
    qdrant_client.upsert(
        collection_name=collection_name,
        points=points,
        wait=True
    )
    print(f"Successfully upserted {len(points)} vectors to Qdrant.")


def run_indexing():
    """
    Main function to run the entire indexing pipeline:
    1. Load documents
    2. Chunk documents
    3. Generate embeddings
    4. Upsert to Qdrant
    """
    print("--- Starting RAG Indexing Service ---")
    
    # 1. Load documents
    documents = load_markdown_documents()
    if not documents:
        print("No documents found to index. Exiting.")
        return
        
    # 2. Chunk documents
    chunked_documents = chunk_documents(documents)
    
    # 3. Generate embeddings
    print(f"Loading embedding model: {EMBEDDING_MODEL}")
    embedding_model = SentenceTransformer(EMBEDDING_MODEL)
    embeddings = generate_embeddings(chunked_documents, embedding_model)
    
    # 4. Upsert to Qdrant
    upsert_to_qdrant(chunked_documents, embeddings)
    
    print("--- RAG Indexing Service Finished ---")

