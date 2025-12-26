from fastapi.concurrency import run_in_threadpool
from qdrant_client import AsyncQdrantClient
from sentence_transformers import SentenceTransformer, CrossEncoder
import google.generativeai as genai

from ..core.config import settings
from .translation_service import TranslationService # Import TranslationService

class RagService:
    def __init__(self, qdrant: AsyncQdrantClient):
        self.qdrant = qdrant
        self.embedding_model = SentenceTransformer(settings.embedding_model_name)
        self.cross_encoder = CrossEncoder(settings.cross_encoder_model_name)
        self.translation_service = TranslationService() # Instantiate TranslationService
        self._cache = {}
        
        # Configure the generative model
        genai.configure(api_key=settings.google_api_key)
        self.generative_model = genai.GenerativeModel('gemini-2.5-flash')

    def _get_docusaurus_url_from_path(self, file_path: str) -> str | None:
        """
        Converts a file system path to a Docusaurus URL.
        Example: "frontend/docs/ros2/week4.md" -> "/humanoid-robotics-book/docs/ros2/week4"
        """
        if "frontend/docs/" in file_path:
            # Extract the part after "frontend/docs/" and remove ".md"
            doc_path = file_path.split("frontend/docs/")[1].replace(".md", "")
            # Construct the full Docusaurus URL
            return f"{settings.base_url}docs/{doc_path}"
        return None

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

    async def query(self, question: str, top_k: int, target_language: str | None = None) -> dict: # Added target_language
        """
        Performs the RAG query:
        1. Embed the question.
        2. Search Qdrant for similar vectors/chunks.
        3. Generate a response using a generative model with the retrieved context.
        4. Translate the response if a target_language is provided.
        """
        # Check cache first
        cache_key = (question, top_k, target_language) # Include target_language in cache key
        if cache_key in self._cache:
            return self._cache[cache_key]

        # 1. Embed the question
        embedding_result = await run_in_threadpool(self.embedding_model.encode, question)
        query_embedding = embedding_result.tolist()
        
        # 2. Search Qdrant
        print(f"DEBUG: Attributes of self.qdrant: {dir(self.qdrant)}") # Add this line
        search_results = await self.qdrant.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
            limit=top_k * 3, # Retrieve more documents for re-ranking
            with_payload=True
        )

        # 3. Re-rank the results
        cross_encoder_inputs = [[question, hit.payload['text']] for hit in search_results]
        cross_encoder_scores = await run_in_threadpool(self.cross_encoder.predict, cross_encoder_inputs)

        # Combine hits with scores and sort
        scored_hits = list(zip(cross_encoder_scores, search_results))
        scored_hits.sort(key=lambda x: x[0], reverse=True)

        # Select the top_k hits
        top_k_hits = scored_hits[:top_k]
        
        # Format retrieved chunks to match SourceDocument model
        retrieved_sources = []
        for _, hit in top_k_hits:
            source_path = hit.payload.get("source", "unknown")
            retrieved_sources.append({
                "source": source_path,
                "content": hit.payload["text"],
                "url": self._get_docusaurus_url_from_path(source_path)
            })
        
        context_chunks_for_llm = [source["content"] for source in retrieved_sources]

        # 4. Generate response using the LLM
        prompt = self._build_prompt(question, context_chunks_for_llm) # Changed 'query' to 'question'
        
        try:
            llm_response = await run_in_threadpool(self.generative_model.generate_content, prompt)
            generated_answer = llm_response.text # Changed 'generated_response' to 'generated_answer'
        except Exception as e:
            print(f"Error during generative model call: {e}")
            generated_answer = "An error occurred while generating the response."
            
        # 5. Translate the generated answer if a target_language is specified
        if target_language and target_language != "en":
            try:
                translated_answer = self.translation_service.translate_text(generated_answer, target_language)
                generated_answer = translated_answer
            except Exception as e:
                print(f"Error during translation: {e}")
                # Fallback to English answer if translation fails
                generated_answer += f" (Translation to {target_language} failed due to: {e})"

        response = {
            "answer": generated_answer, # Changed 'response' to 'answer'
            "sources": retrieved_sources # Changed 'retrieved_chunks' to 'sources'
        }

        # Store in cache
        self._cache[cache_key] = response

        return response