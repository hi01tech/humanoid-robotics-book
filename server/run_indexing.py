import argparse
from app.services.indexing_service import IndexingService
from app.core.config import settings
from app.db.vector_store import get_qdrant_client

def main():
    """
    Main function to run the indexing process.
    Connects to Qdrant and starts the IndexingService.
    """
    parser = argparse.ArgumentParser(description="Index documentation into Qdrant.")
    parser.add_argument(
        "--docs_path",
        type=str,
        default="docs",
        help="Path to the documentation directory, relative to the project root."
    )
    args = parser.parse_args()

    print("--- Starting Indexing Process ---")
    
    # Initialize Qdrant client
    qdrant_client = get_qdrant_client()
    
    # Initialize the indexing service
    indexing_service = IndexingService(
        qdrant_client=qdrant_client,
        collection_name=settings.qdrant_collection_name
    )
    
    # Run the indexing process
    indexing_service.index_documents(docs_path=args.docs_path)
    
    print("--- Indexing Process Finished ---")

if __name__ == "__main__":
    main()