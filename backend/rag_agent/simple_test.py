#!/usr/bin/env python3
"""
Simple test to verify the RAG agent is working properly
"""
import os
import sys
import asyncio
from unittest.mock import Mock, patch, AsyncMock

# Add the current directory to Python path
sys.path.insert(0, '.')

from main import app, retrieve_contexts, generate_response_with_context, validate_environment_variables

def test_environment_validation():
    """Test that environment variables are validated properly"""
    print("Testing environment variable validation...")
    try:
        validate_environment_variables()
        print("[PASS] Environment variables validation passed")
        return True
    except Exception as e:
        print(f"[FAIL] Environment variables validation failed: {e}")
        return False

def test_app_routes():
    """Test that FastAPI app has the expected routes"""
    print("Testing FastAPI app routes...")
    expected_routes = {"/", "/query", "/health", "/config"}
    actual_routes = {route.path for route in app.routes if hasattr(route, 'path')}

    if expected_routes.issubset(actual_routes):
        print("[PASS] All expected routes are present")
        return True
    else:
        print(f"[FAIL] Missing routes. Expected: {expected_routes}, Actual: {actual_routes}")
        return False

async def test_retrieve_contexts():
    """Test the retrieve_contexts function with mocked dependencies"""
    print("Testing retrieve_contexts function...")

    # Mock Qdrant client
    mock_qdrant_client = Mock()
    mock_search_result = [
        Mock(id="test_id_1", score=0.9, payload={"content": "Test content 1", "source_api": "test_source"})
    ]
    mock_qdrant_client.search.return_value = mock_search_result

    with patch('main.cohere.Client') as mock_cohere_class:
        mock_cohere_response = Mock()
        mock_cohere_response.embeddings = [[0.1, 0.2, 0.3]]
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.return_value = mock_cohere_response
        mock_cohere_class.return_value = mock_cohere_instance

        try:
            contexts = await retrieve_contexts("test query", 5, mock_qdrant_client)
            if len(contexts) == 1 and contexts[0].id == "test_id_1":
                print("[PASS] retrieve_contexts function works correctly")
                return True
            else:
                print(f"[FAIL] retrieve_contexts returned unexpected result: {contexts}")
                return False
        except Exception as e:
            print(f"[FAIL] retrieve_contexts function failed: {e}")
            return False

async def test_generate_response_with_context():
    """Test the generate_response_with_context function with mocked dependencies"""
    print("Testing generate_response_with_context function...")

    # Create test contexts
    from pydantic import BaseModel
    from typing import Dict, Any

    class MockRetrievedContext(BaseModel):
        id: str
        content: str
        score: float
        metadata: Dict[str, Any]

    test_contexts = [
        MockRetrievedContext(
            id="test_id_1",
            content="This is test content 1",
            score=0.9,
            metadata={"source_api": "test_source"}
        )
    ]

    with patch('main.openai.OpenAI') as mock_openai_class:
        mock_openai_instance = Mock()
        mock_choice = Mock()
        mock_choice.message.content = "This is the generated answer."
        mock_response = Mock()
        mock_response.choices = [mock_choice]
        mock_openai_instance.chat.completions.create.return_value = mock_response
        mock_openai_class.return_value = mock_openai_instance

        try:
            answer = await generate_response_with_context("test query", test_contexts, mock_openai_instance)
            if "generated answer" in answer:
                print("[PASS] generate_response_with_context function works correctly")
                return True
            else:
                print(f"[FAIL] generate_response_with_context returned unexpected result: {answer}")
                return False
        except Exception as e:
            print(f"[FAIL] generate_response_with_context function failed: {e}")
            return False

async def run_all_tests():
    """Run all tests"""
    print("Running RAG Agent validation tests...\n")

    tests = [
        ("Environment validation", test_environment_validation),
        ("App routes", test_app_routes),
        ("Retrieve contexts", test_retrieve_contexts),
        ("Generate response", test_generate_response_with_context),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"Running {test_name} test...")
        if callable(test_func) and asyncio.iscoroutinefunction(test_func):
            result = await test_func()
        else:
            result = test_func()
        results.append((test_name, result))
        print()

    print("Test Results:")
    all_passed = True
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"  {test_name}: {status}")
        if not result:
            all_passed = False

    print(f"\nOverall result: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")
    return all_passed

if __name__ == "__main__":
    success = asyncio.run(run_all_tests())
    sys.exit(0 if success else 1)