from anyio import to_thread
import google.generativeai as genai
from sentence_transformers import SentenceTransformer
from app.core.config import settings
from app.db.vector_store import get_qdrant_client
from app.models.rag import QueryResponse, SourceDocument
from qdrant_client.models import SearchRequest

# --- Configuration ---
EMBEDDING_MODEL_NAME = 'all-MiniLM-L6-v2'
GEMINI_MODEL_NAME = 'gemini-1.5-flash'

# --- Initialize models ---
print("Loading embedding model...")
embedding_model = SentenceTransformer(EMBEDDING_MODEL_NAME)
print("Embedding model loaded.")

print("Configuring Gemini...")
genai.configure(api_key=settings.gemini_api_key)
gemini_model = genai.GenerativeModel(GEMINI_MODEL_NAME)
print("Gemini configured.")


def build_prompt(query: str, context: list[dict]) -> str:
    """Builds a prompt for the LLM from the query and context."""
    context_str = "\n\n---\n\n".join([doc['payload']['text'] for doc in context])
    
    prompt = f"""
    You are an expert Q&A system for the 'Humanoid Robotics Textbook'.
    Your task is to answer the user's question based *only* on the provided context.
    If the context does not contain the answer, state that you cannot answer the question with the given information.
    Do not make up information.
    
    Here is the context:
    
    {context_str}
    
    ---
    
    Here is the user's question:
    
    \"{query}\"      
    Answer:
    """
    return prompt

async def get_rag_answer(query: str, top_k: int) -> QueryResponse:
    print(f"Received query: {query}")
    
    # 1. Embed the query (Run in separate thread to prevent blocking)
    print("Embedding user query...")
    # The result of run_sync is the array, which we then convert to a list.
    query_embedding_array = await to_thread.run_sync(embedding_model.encode, query)
    query_embedding = query_embedding_array.tolist()
    
    # 2. Search Qdrant
    print("Searching Qdrant for relevant documents...")
    qdrant_client = get_qdrant_client()
    
    # --- DEBUGGING BLOCK ---
    print(f"DEBUG: Client Type: {type(qdrant_client)}")
    print(f"DEBUG: Client Dir: {dir(qdrant_client)}")
    # -----------------------

    # Explicit Search Request
    search_request = SearchRequest(
        vector=query_embedding,
        limit=top_k,
        with_payload=True
    )

    try:
        search_results = qdrant_client.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=search_request.vector,
            limit=search_request.limit,
            with_payload=search_request.with_payload
        )
    except AttributeError:
        # Fallback for some client versions that prefer .query_points or .retrieve
        print("DEBUG: .search() failed, attempting fallback...")
        search_results = qdrant_client.query_points(
            collection_name=settings.qdrant_collection_name,
            query=search_request.vector,
            limit=search_request.limit,
            with_payload=search_request.with_payload
        ).points

    # Convert results for prompt
    context_for_prompt = [result.model_dump() for result in search_results]
    
    # 3. Generate response using Gemini
    print("Building prompt and querying Gemini...")
    prompt = build_prompt(query, context_for_prompt)
    
    try:
        response = gemini_model.generate_content(prompt)
        final_answer = response.text
        print("Received response from Gemini.")
    except Exception as e:
        print(f"Error calling Gemini API: {e}")
        final_answer = "Error: Could not generate an answer from the language model."

    # Prepare source documents
    sources = [
        SourceDocument(
            source=hit.payload['metadata']['source'],
            content=hit.payload['text']
        )
        for hit in search_results
    ]
    
    return QueryResponse(answer=final_answer, sources=sources)