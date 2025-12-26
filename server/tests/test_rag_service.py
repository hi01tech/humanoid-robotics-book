import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from server.app.services.rag_service import RagService

@pytest.fixture
def mock_qdrant_client():
    qdrant_mock = MagicMock()
    # Mock the search method to return a list of mock hits
    mock_hit = MagicMock()
    mock_hit.payload = {"source": "test_source", "text": "test_content"}
    qdrant_mock.search.return_value = [mock_hit, mock_hit, mock_hit]
    return qdrant_mock

@pytest.fixture
def mock_sentence_transformer():
    mock = MagicMock()
    mock.encode.return_value.tolist.return_value = [0.1, 0.2, 0.3]
    return mock

@pytest.fixture
def mock_cross_encoder():
    mock = MagicMock()
    mock.predict.return_value = [0.9, 0.8, 0.7]
    return mock

@pytest.fixture
def mock_genai():
    mock_genai = MagicMock()
    mock_genai.GenerativeModel.return_value.generate_content.return_value.text = "Generated Answer"
    return mock_genai

@pytest.fixture
def rag_service(mock_qdrant_client, mock_sentence_transformer, mock_cross_encoder, mock_genai):
    with patch('server.app.services.rag_service.SentenceTransformer', return_value=mock_sentence_transformer), \
         patch('server.app.services.rag_service.CrossEncoder', return_value=mock_cross_encoder), \
         patch('server.app.services.rag_service.genai', mock_genai):
        service = RagService(mock_qdrant_client)
    return service



def test_rag_service_instantiation(rag_service):
    """
    Tests that the RagService can be instantiated.
    """
    assert rag_service is not None

@pytest.mark.asyncio
async def test_query_successful_path(rag_service, mock_qdrant_client):
    """
    Tests a successful query through the RAG service.
    """
    question = "What is a digital twin?"
    top_k = 1

    # Mock run_in_threadpool
    with patch('server.app.services.rag_service.run_in_threadpool', new_callable=AsyncMock) as mock_run_in_threadpool:
        # Define side effects for the mock
        mock_run_in_threadpool.side_effect = [
            [0.1, 0.2, 0.3],  # Return for embedding_model.encode
            [0.9, 0.8, 0.7],  # Return for cross_encoder.predict
            MagicMock(text="Generated Answer") # Return for generative_model.generate_content
        ]
        
        response = await rag_service.query(question, top_k)

        assert response["answer"] == "Generated Answer"
        assert len(response["sources"]) == top_k
        assert response["sources"][0]["source"] == "test_source"
        mock_qdrant_client.search.assert_called_once()


@pytest.mark.asyncio
async def test_caching_mechanism(rag_service):
    """
    Tests that the caching mechanism works as expected.
    """
    question = "What is a digital twin?"
    top_k = 1

    with patch('server.app.services.rag_service.run_in_threadpool', new_callable=AsyncMock) as mock_run_in_threadpool:
        mock_run_in_threadpool.side_effect = [
            [0.1, 0.2, 0.3], [0.9, 0.8, 0.7], MagicMock(text="First Answer")
        ]
        # First call - should execute the full path
        first_response = await rag_service.query(question, top_k)
        assert first_response["answer"] == "First Answer"
        assert mock_run_in_threadpool.call_count == 3

        # Second call - should return the cached result
        second_response = await rag_service.query(question, top_k)
        assert second_response["answer"] == "First Answer"
        # The call count should not have increased
        assert mock_run_in_threadpool.call_count == 3
