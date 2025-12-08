import google.generativeai as genai
from sentence_transformers import SentenceTransformer
from app.core.config import settings
from app.db.vector_store import get_qdrant_client
from app.models.rag import QueryResponse, SourceDocument

# --- Configuration ---
EMBEDDING_MODEL_NAME = 'all-MiniLM-L6-v2'
GEMINI_MODEL_NAME = 'gemini-1.5-flash'

# --- Initialize models ---
# It's better to load these once and reuse them.
# In a real production app, you might use a dependency injection system
# or a class-based service to manage the model lifecycle.
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
    """
    Performs the full RAG pipeline:
    1. Embeds the query.
    2. Searches Qdrant.
    3. Generates a response using Gemini.
    """
    print(f"Received query: {query}")
    
    # 1. Embed the query
    print("Embedding user query...")
    query_embedding = embedding_model.encode(query).tolist()
    
    # 2. Search Qdrant
    print("Searching Qdrant for relevant documents...")
    qdrant_client = get_qdrant_client()
    search_results = qdrant_client.search(
        collection_name=settings.qdrant_collection_name,
        query_vector=query_embedding,
        limit=top_k,
        with_payload=True  # Ensure we get the payload
    )
    
    # Convert ScoredPoint objects to dictionaries for prompt building
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

    # Prepare source documents for the final response
    sources = [
        SourceDocument(
            source=hit.payload['metadata']['source'],
            content=hit.payload['text']
        )
        for hit in search_results
    ]
    
    return QueryResponse(answer=final_answer, sources=sources)