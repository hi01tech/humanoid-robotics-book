import os
import glob
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_core.documents import Document
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import UpdateStatus

from ..core.config import settings

class IndexingService:
    def __init__(self, qdrant_client: QdrantClient, collection_name: str):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name
        self.embedding_model = SentenceTransformer(settings.embedding_model_name)
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=300,
            length_function=len,
        )

    def _load_documents(self, docs_path: str) -> list[Document]:
        """Loads all markdown files from the specified path."""
        # Use a relative path from the server directory
        # server/../docs/**/*.md
        full_path = os.path.join(os.path.dirname(__file__), '..', '..', docs_path, '**', '*.md')
        
        markdown_files = glob.glob(full_path, recursive=True)
        
        documents = []
        for file_path in markdown_files:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                # Use file path as a metadata source
                doc = Document(page_content=content, metadata={"source": file_path})
                documents.append(doc)
        return documents

    def _split_documents(self, documents: list[Document]) -> list[Document]:
        """Splits the documents into smaller chunks."""
        return self.text_splitter.split_documents(documents)

    def _create_collection_if_not_exists(self):
        """Creates the Qdrant collection if it doesn't already exist."""
        try:
            self.qdrant_client.get_collection(collection_name=self.collection_name)
            print(f"Collection '{self.collection_name}' already exists.")
        except Exception:
            print(f"Collection '{self.collection_name}' not found. Creating new collection.")
            self.qdrant_client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.embedding_model.get_sentence_embedding_dimension(),
                    distance=models.Distance.COSINE
                ),
            )

    def index_documents(self, docs_path: str = "docs"):
        """The main method to load, split, and index the documents."""
        self._create_collection_if_not_exists()
        
        print("Loading documents...")
        documents = self._load_documents(docs_path)
        print(f"Loaded {len(documents)} documents.")
        
        print("Splitting documents...")
        chunks = self._split_documents(documents)
        print(f"Split documents into {len(chunks)} chunks.")
        
        print("Generating embeddings and indexing...")
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=[
                models.PointStruct(
                    id=idx,
                    vector=self.embedding_model.encode(chunk.page_content).tolist(),
                    payload={
                        "text": chunk.page_content,
                        "source": chunk.metadata["source"]
                    }
                ) for idx, chunk in enumerate(chunks)
            ]
        )
        print("Indexing complete.")