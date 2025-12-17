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

    def _build_prompt(self, question: str, context_chunks: list[str]) -> str:
        """Builds a grounded prompt for the generative model."""
        
        context = "\n\n".join(context_chunks)
        
        prompt = f"""
        You are a helpful assistant for the 'Physical AI & Humanoid Robotics' textbook.
        Answer the following question based ONLY on the context provided below.
        If the context does not contain the answer, state that you cannot answer the question with the provided context.

        CONTEXT:
        {context}

        QUESTION:
        {question}
        """
        return prompt

    async def query(self, question: str, top_k: int) -> dict: # Changed 'query' to 'question'
        """
        Performs the RAG query:
        1. Embed the question.
        2. Search Qdrant for similar vectors/chunks.
        3. Generate a response using a generative model with the retrieved context.
        """
        # 1. Embed the question
        query_embedding = self.embedding_model.encode(question).tolist()
        
        # 2. Search Qdrant
        search_results = self.qdrant.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )
        
        # Format retrieved chunks to match SourceDocument model
        retrieved_sources = []
        for hit in search_results:
            retrieved_sources.append({
                "source": hit.payload.get("source", "unknown"),
                "content": hit.payload["text"]
            })
        
        context_chunks_for_llm = [source["content"] for source in retrieved_sources]

        # 3. Generate response using the LLM
        prompt = self._build_prompt(question, context_chunks_for_llm) # Changed 'query' to 'question'
        
        try:
            llm_response = self.generative_model.generate_content(prompt)
            generated_answer = llm_response.text # Changed 'generated_response' to 'generated_answer'
        except Exception as e:
            print(f"Error during generative model call: {e}")
            generated_answer = "An error occurred while generating the response."
            
        return {
            "answer": generated_answer, # Changed 'response' to 'answer'
            "sources": retrieved_sources # Changed 'retrieved_chunks' to 'sources'
        }