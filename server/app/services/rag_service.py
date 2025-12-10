from qdrant_client import QdrantClient, models
from sentence_transformers import SentenceTransformer
import google.generativeai as genai

from app.core.config import settings

class RagService:
    def __init__(self, qdrant: QdrantClient):
        self.qdrant = qdrant
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
        
        # Configure the generative model
        genai.configure(api_key=settings.google_api_key)
        self.generative_model = genai.GenerativeModel('gemini-pro')

    def _build_prompt(self, query: str, context_chunks: list[str]) -> str:
        """Builds a grounded prompt for the generative model."""
        
        context = "\n\n".join(context_chunks)
        
        prompt = f"""
You are a helpful assistant for the 'Physical AI & Humanoid Robotics' textbook.
Answer the following question based ONLY on the context provided below.
If the context does not contain the answer, state that you cannot answer the question with the provided context.

CONTEXT:
{context}

QUESTION:
{query}
        """
        return prompt

    async def query(self, query: str, top_k: int) -> dict:
        """
        Performs the RAG query:
        1. Embed the query.
        2. Search Qdrant for similar vectors/chunks.
        3. Generate a response using a generative model with the retrieved context.
        """
        # 1. Embed the query
        query_embedding = self.embedding_model.encode(query).tolist()
        
        # 2. Search Qdrant
        search_results = self.qdrant.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )
        
        retrieved_chunks = [hit.payload["text"] for hit in search_results]
        
        # 3. Generate response using the LLM
        prompt = self._build_prompt(query, retrieved_chunks)
        
        try:
            llm_response = self.generative_model.generate_content(prompt)
            generated_response = llm_response.text
        except Exception as e:
            print(f"Error during generative model call: {e}")
            generated_response = "An error occurred while generating the response."
            
        return {
            "query": query,
            "retrieved_chunks": retrieved_chunks,
            "response": generated_response
        }
