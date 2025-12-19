"""
Test suite for the RAG Agent API
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, AsyncMock
from rag_agent.main import app, retrieve_contexts, generate_response_with_context
from pydantic import BaseModel
from typing import List, Dict, Any


class MockRetrievedContext(BaseModel):
    id: str
    content: str
    score: float
    metadata: Dict[str, Any]


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    return TestClient(app)


def test_root_endpoint(client):
    """Test the root endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["message"] == "RAG Agent API is running"


def test_health_endpoint(client):
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["service"] == "RAG Agent API"


@patch('rag_agent.main.retrieve_contexts', new_callable=AsyncMock)
@patch('rag_agent.main.generate_response_with_context', new_callable=AsyncMock)
def test_query_endpoint_success(mock_gen_response, mock_retrieve_contexts, client):
    """Test the query endpoint with successful response"""
    # Mock the retrieve_contexts function
    mock_contexts = [
        MockRetrievedContext(
            id="test_id_1",
            content="This is test content 1",
            score=0.9,
            metadata={"source_api": "test_source_1"}
        ),
        MockRetrievedContext(
            id="test_id_2",
            content="This is test content 2",
            score=0.8,
            metadata={"source_api": "test_source_2"}
        )
    ]
    mock_retrieve_contexts.return_value = mock_contexts

    # Mock the generate_response_with_context function
    mock_gen_response.return_value = "This is the generated answer based on the context."

    # Make the request
    response = client.post(
        "/query",
        json={
            "query": "What is this book about?",
            "top_k": 5
        }
    )

    assert response.status_code == 200
    data = response.json()

    # Verify the response structure
    assert "query" in data
    assert "answer" in data
    assert "retrieved_contexts" in data
    assert "sources" in data

    assert data["query"] == "What is this book about?"
    assert data["answer"] == "This is the generated answer based on the context."
    assert len(data["retrieved_contexts"]) == 2
    assert len(data["sources"]) == 2


@patch('rag_agent.main.retrieve_contexts', new_callable=AsyncMock)
def test_query_endpoint_no_contexts(mock_retrieve_contexts, client):
    """Test the query endpoint when no contexts are found"""
    # Mock the retrieve_contexts function to return empty list
    mock_retrieve_contexts.return_value = []

    # Make the request
    response = client.post(
        "/query",
        json={
            "query": "What is this book about?",
            "top_k": 5
        }
    )

    assert response.status_code == 200
    data = response.json()

    # Verify the response for no contexts found
    assert data["query"] == "What is this book about?"
    assert "couldn't find any relevant content" in data["answer"]
    assert len(data["retrieved_contexts"]) == 0
    assert len(data["sources"]) == 0


def test_config_endpoints(client):
    """Test the configuration endpoints"""
    # Test getting default config
    response = client.get("/config")
    assert response.status_code == 200
    data = response.json()
    assert "retrieval_depth" in data
    assert "response_style" in data
    assert "temperature" in data

    # Test updating config
    new_config = {
        "retrieval_depth": 10,
        "response_style": "detailed",
        "temperature": 0.5
    }

    response = client.post("/config", json=new_config)
    assert response.status_code == 200
    data = response.json()
    assert data["message"] == "Configuration updated successfully"
    assert data["config"] == new_config


@pytest.mark.asyncio
@patch('cohere.Client')
@patch('qdrant_client.QdrantClient')
async def test_retrieve_contexts(mock_qdrant_client, mock_cohere_client):
    """Test the retrieve_contexts function"""
    # Mock Cohere client
    mock_cohere_response = Mock()
    mock_cohere_response.embeddings = [[0.1, 0.2, 0.3]]
    mock_cohere_instance = Mock()
    mock_cohere_instance.embed.return_value = mock_cohere_response
    mock_cohere_client.return_value = mock_cohere_instance

    # Mock Qdrant client
    mock_qdrant_instance = Mock()
    mock_search_result = [
        Mock(id="test_id_1", score=0.9, payload={"content": "Test content 1", "source_api": "test_source"})
    ]
    mock_qdrant_instance.search.return_value = mock_search_result
    mock_qdrant_client.return_value = mock_qdrant_instance

    # Call the function
    contexts = await retrieve_contexts("test query", 5, mock_qdrant_instance)

    # Verify the result
    assert len(contexts) == 1
    assert contexts[0].id == "test_id_1"
    assert contexts[0].content == "Test content 1"
    assert contexts[0].score == 0.9
    assert contexts[0].metadata["source_api"] == "test_source"


@pytest.mark.asyncio
@patch('openai.OpenAI')
async def test_generate_response_with_context(mock_openai_client):
    """Test the generate_response_with_context function"""
    # Mock OpenAI client
    mock_openai_instance = Mock()
    mock_choice = Mock()
    mock_choice.message.content = "This is the generated answer."
    mock_response = Mock()
    mock_response.choices = [mock_choice]
    mock_openai_instance.chat.completions.create.return_value = mock_response
    mock_openai_client.return_value = mock_openai_instance

    # Create test contexts
    test_contexts = [
        MockRetrievedContext(
            id="test_id_1",
            content="This is test content 1",
            score=0.9,
            metadata={"source_api": "test_source"}
        )
    ]

    # Call the function
    answer = await generate_response_with_context("test query", test_contexts, mock_openai_instance)

    # Verify the result
    assert answer == "This is the generated answer."
    # Verify that the OpenAI API was called
    mock_openai_instance.chat.completions.create.assert_called_once()


def test_error_handling_query_endpoint(client):
    """Test error handling in the query endpoint"""
    with patch('rag_agent.main.retrieve_contexts', side_effect=Exception("Test error")):
        response = client.post(
            "/query",
            json={
                "query": "What is this book about?",
                "top_k": 5
            }
        )
        assert response.status_code == 500