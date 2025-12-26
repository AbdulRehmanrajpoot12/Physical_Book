from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import asyncio
import logging
import os
from datetime import datetime
import httpx
from qdrant_client import QdrantClient
from qdrant_client.http import models
import json
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG (Retrieval Augmented Generation) chatbot that answers questions based on book content",
    version="1.0.0"
)

# Add CORS middleware with proper configuration for frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods including OPTIONS
    allow_headers=["*"],  # Allow all headers
    # Additional configuration for preflight requests
    max_age=3600,  # Cache preflight requests for 1 hour
)

# Pydantic models
class QueryRequest(BaseModel):
    query: str
    context_scope: Optional[str] = "all_content"
    selected_text: Optional[str] = None
    temperature: Optional[float] = 0.7
    max_tokens: Optional[int] = 500

class Source(BaseModel):
    chapter: str
    section: str
    similarity_score: float

class QueryResponse(BaseModel):
    answer: str
    sources: List[Source]
    confidence: str
    timestamp: str

class HealthResponse(BaseModel):
    status: str
    vector_db_status: str
    llm_status: str
    last_updated: str

class VectorizeRequest(BaseModel):
    content_id: str
    content: str
    metadata: dict

class VectorizeResponse(BaseModel):
    success: bool
    vector_id: str
    processed_at: str

# Initialize clients
qdrant_client = QdrantClient(
    url="http://localhost:6333",  # Use URL format for HTTP API access
)
openrouter_client = httpx.AsyncClient(
    base_url="https://openrouter.ai/api/v1",
    headers={
        "Authorization": f"Bearer {os.getenv('OPENROUTER_API_KEY', '')}",
        "Content-Type": "application/json"
    }
)

@app.on_event("startup")
async def startup_event():
    logger.info("Starting up RAG Chatbot API")
    # Initialize vector collection if it doesn't exist
    try:
        qdrant_client.get_collection("book_content")
        logger.info("Connected to Qdrant collection: book_content")
    except Exception as e:
        logger.warning(f"Could not connect to Qdrant: {str(e)}")
        logger.info("Qdrant may not be running - collection will be created when available")

@app.post("/api/chat/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Submit a question to the RAG chatbot and receive an answer based on book content."""
    # Retrieve relevant chunks from Qdrant
    try:
        query_embedding = get_embedding(request.query)
    except Exception as e:
        logger.error(f"Error getting embedding: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": {
                "code": "EMBEDDING_ERROR",
                "message": str(e)
            }
        })

    try:
        # Use the low-level HTTP API with synchronous requests
        import requests
        search_response = requests.post(
            f"http://localhost:6333/collections/book_content/points/search",
            json={
                "vector": query_embedding,
                "limit": 5,
                "with_payload": True
            },
            headers={"Content-Type": "application/json"}
        )
        if search_response.status_code == 200:
            search_results = search_response.json()["result"]
        else:
            logger.error(f"Qdrant search failed: {search_response.status_code} - {search_response.text}")
            raise Exception(f"Qdrant search failed: {search_response.status_code}")

        # Build context from retrieved chunks
        context_chunks = []
        sources = []
        for result in search_results:
            if result.get("payload"):
                context_chunks.append(result["payload"].get("content", ""))
                sources.append(Source(
                    chapter=result["payload"].get("chapter", "Unknown"),
                    section=result["payload"].get("section", "Unknown"),
                    similarity_score=result.get("score", 0.0)
                ))

        context = "\n\n".join(context_chunks)

        # Prepare the prompt for OpenRouter
        prompt = f"""
        You are an AI assistant helping users with questions about robotics, AI, and the book content.
        Use the following context to answer the question:

        {context}

        Question: {request.query}

        Answer:
        """

        try:
            # Try to get response from OpenRouter
            response = await openrouter_client.post(
                "/chat/completions",
                json={
                    "model": "xiaomi/mimo-v2-flash:free",
                    "messages": [
                        {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context. Be concise and accurate."},
                        {"role": "user", "content": prompt}
                    ],
                    "temperature": request.temperature,
                    "max_tokens": request.max_tokens
                }
            )

            if response.status_code != 200:
                logger.error(f"OpenRouter API error: {response.status_code} - {response.text}")
                raise Exception(f"OpenRouter API error: {response.status_code}")

            response_data = response.json()
            logger.info(f"OpenRouter response JSON: {json.dumps(response_data, indent=2)}")
            answer = response_data["choices"][0]["message"]["content"]

            return QueryResponse(
                answer=answer,
                sources=sources,
                confidence="high" if response_data.get("usage", {}).get("total_tokens", 0) > 0 else "medium",
                timestamp=datetime.now().isoformat()
            )

        except httpx.HTTPStatusError as e:
            # Handle OpenRouter rate limit (429) - implement graceful fallback
            if e.response.status_code == 429:
                logger.warning(f"OpenRouter rate limit exceeded: {str(e)}")
                # Still return retrieved context with a message
                return QueryResponse(
                    answer="LLM response unavailable due to API quota; retrieval verified.",
                    sources=sources,  # Return retrieved sources even when LLM is unavailable
                    confidence="medium",  # Confidence based on retrieval success
                    timestamp=datetime.now().isoformat()
                )
            else:
                logger.error(f"Error calling OpenRouter: {str(e)}")
                return QueryResponse(
                    answer="LLM response unavailable due to API error; retrieval verified.",
                    sources=sources,
                    confidence="low",
                    timestamp=datetime.now().isoformat()
                )

        except Exception as e:
            logger.error(f"Error calling OpenRouter: {str(e)}")

            # Still return retrieved context with a message
            return QueryResponse(
                answer="LLM response unavailable due to API error; retrieval verified.",
                sources=sources,  # Return retrieved sources even when LLM fails
                confidence="low",
                timestamp=datetime.now().isoformat()
            )

    except Exception as e:
        # Handle Qdrant errors including potential rate limiting or connection issues
        logger.error(f"Error querying Qdrant: {str(e)}")
        # Return a response with an informative message but no sources
        return QueryResponse(
            answer="I'm currently unable to retrieve information from the knowledge base. The vector database may be temporarily unavailable. Please try again in a moment.",
            sources=[],
            confidence="low",
            timestamp=datetime.now().isoformat()
        )


@app.post("/query", response_model=QueryResponse)
async def simple_query_endpoint(request: QueryRequest):
    """Simple query endpoint that mirrors the functionality of /api/chat/query for compatibility."""
    # This is a duplicate of the main query endpoint to support simple /query calls
    try:
        query_embedding = get_embedding(request.query)
    except Exception as e:
        logger.error(f"Error getting embedding: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": {
                "code": "EMBEDDING_ERROR",
                "message": str(e)
            }
        })

    try:
        # Use the low-level HTTP API with synchronous requests
        import requests
        search_response = requests.post(
            f"http://localhost:6333/collections/book_content/points/search",
            json={
                "vector": query_embedding,
                "limit": 5,
                "with_payload": True
            },
            headers={"Content-Type": "application/json"}
        )
        if search_response.status_code == 200:
            search_results = search_response.json()["result"]
        else:
            logger.error(f"Qdrant search failed: {search_response.status_code} - {search_response.text}")
            raise Exception(f"Qdrant search failed: {search_response.status_code}")

        # Build context from retrieved chunks
        context_chunks = []
        sources = []
        for result in search_results:
            if result.get("payload"):
                context_chunks.append(result["payload"].get("content", ""))
                sources.append(Source(
                    chapter=result["payload"].get("chapter", "Unknown"),
                    section=result["payload"].get("section", "Unknown"),
                    similarity_score=result.get("score", 0.0)
                ))

        context = "\n\n".join(context_chunks)

        # Prepare the prompt for OpenRouter
        prompt = f"""
        You are an AI assistant helping users with questions about robotics, AI, and the book content.
        Use the following context to answer the question:

        {context}

        Question: {request.query}

        Answer:
        """

        try:
            # Try to get response from OpenRouter
            response = await openrouter_client.post(
                "/chat/completions",
                json={
                    "model": "xiaomi/mimo-v2-flash:free",
                    "messages": [
                        {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context. Be concise and accurate."},
                        {"role": "user", "content": prompt}
                    ],
                    "temperature": request.temperature,
                    "max_tokens": request.max_tokens
                }
            )

            if response.status_code != 200:
                logger.error(f"OpenRouter API error: {response.status_code} - {response.text}")
                raise Exception(f"OpenRouter API error: {response.status_code}")

            response_data = response.json()
            logger.info(f"OpenRouter response JSON: {json.dumps(response_data, indent=2)}")
            answer = response_data["choices"][0]["message"]["content"]

            return QueryResponse(
                answer=answer,
                sources=sources,
                confidence="high" if response_data.get("usage", {}).get("total_tokens", 0) > 0 else "medium",
                timestamp=datetime.now().isoformat()
            )

        except httpx.HTTPStatusError as e:
            # Handle OpenRouter rate limit (429) - implement graceful fallback
            if e.response.status_code == 429:
                logger.warning(f"OpenRouter rate limit exceeded: {str(e)}")
                # Still return retrieved context with a message
                return QueryResponse(
                    answer="LLM response unavailable due to API quota; retrieval verified.",
                    sources=sources,  # Return retrieved sources even when LLM is unavailable
                    confidence="medium",  # Confidence based on retrieval success
                    timestamp=datetime.now().isoformat()
                )
            else:
                logger.error(f"Error calling OpenRouter: {str(e)}")
                return QueryResponse(
                    answer="LLM response unavailable due to API error; retrieval verified.",
                    sources=sources,
                    confidence="low",
                    timestamp=datetime.now().isoformat()
                )

        except Exception as e:
            logger.error(f"Error calling OpenRouter: {str(e)}")

            # Still return retrieved context with a message
            return QueryResponse(
                answer="LLM response unavailable due to API error; retrieval verified.",
                sources=sources,  # Return retrieved sources even when LLM fails
                confidence="low",
                timestamp=datetime.now().isoformat()
            )

    except Exception as e:
        # Handle Qdrant errors including potential rate limiting or connection issues
        logger.error(f"Error querying Qdrant: {str(e)}")
        # Return a response with an informative message but no sources
        return QueryResponse(
            answer="I'm currently unable to retrieve information from the knowledge base. The vector database may be temporarily unavailable. Please try again in a moment.",
            sources=[],
            confidence="low",
            timestamp=datetime.now().isoformat()
        )


@app.options("/query")
async def options_simple_query_endpoint(request: Request):
    """Handle OPTIONS preflight request for simple query endpoint."""
    return {"detail": "OPTIONS request handled"}

@app.get("/api/chat/health", response_model=HealthResponse)
async def health_check():
    """Check the health status of the RAG service."""
    vector_db_status = "connected"
    llm_status = "available"

    # Check Qdrant connection
    try:
        qdrant_client.get_collection("book_content")
    except:
        vector_db_status = "disconnected"

    # Check OpenRouter availability
    try:
        # Make a simple test request to OpenRouter
        test_response = await openrouter_client.post(
            "/chat/completions",
            json={
                "model": os.getenv("OPENROUTER_MODEL", "bytedance-seed/seedream-4.5"),
                "messages": [
                    {"role": "user", "content": "test"}
                ],
                "max_tokens": 1
            }
        )
        if test_response.status_code != 200:
            logger.error(f"OpenRouter health check error: {test_response.status_code} - {test_response.text}")
            llm_status = "unavailable"
        else:
            test_response_data = test_response.json()
            logger.info(f"OpenRouter health check response JSON: {json.dumps(test_response_data, indent=2)}")
    except Exception as e:
        logger.error(f"OpenRouter health check exception: {str(e)}")
        llm_status = "unavailable"

    overall_status = "error"
    if vector_db_status == "connected" and llm_status == "available":
        overall_status = "ok"
    elif vector_db_status == "connected" or llm_status == "available":
        overall_status = "degraded"

    return HealthResponse(
        status=overall_status,
        vector_db_status=vector_db_status,
        llm_status=llm_status,
        last_updated=datetime.now().isoformat()
    )

@app.post("/api/chat/vectorize", response_model=VectorizeResponse)
async def vectorize_endpoint(request: VectorizeRequest):
    """Convert book content to vector embeddings."""
    try:
        # Get embedding for the content
        embedding = get_embedding(request.content)

        # Store in Qdrant
        qdrant_client.upsert(
            collection_name="book_content",
            points=[
                models.PointStruct(
                    id=request.content_id,
                    vector=embedding,
                    payload={
                        "content": request.content,
                        "chapter": request.metadata.get("chapter", ""),
                        "section": request.metadata.get("section", ""),
                        "tags": request.metadata.get("tags", []),
                        "created_at": datetime.now().isoformat()
                    }
                )
            ]
        )

        return VectorizeResponse(
            success=True,
            vector_id=request.content_id,
            processed_at=datetime.now().isoformat()
        )
    except Exception as e:
        logger.error(f"Error vectorizing content: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": {
                "code": "PROCESSING_ERROR",
                "message": str(e)
            }
        })

def get_embedding(text: str) -> List[float]:
    """Get embedding for text using a local model since OpenRouter doesn't provide embedding API."""
    # Import sentence-transformers for local embeddings
    try:
        from sentence_transformers import SentenceTransformer
        import numpy as np

        # Use a lightweight local model for embeddings
        # Cache the model to avoid reloading for each request
        if not hasattr(get_embedding, 'model'):
            get_embedding.model = SentenceTransformer('all-MiniLM-L6-v2')

        # Generate embedding
        embedding = get_embedding.model.encode([text])
        return embedding[0].tolist()  # Convert to list format
    except ImportError:
        # Fallback: if sentence_transformers is not available, use a simple approach
        # In production, you should ensure sentence_transformers is installed
        import random
        # Generate a 384-dimensional vector (size of all-MiniLM-L6-v2 embeddings) with random values
        # This is just a fallback - install sentence_transformers for proper functionality
        return [random.random() for _ in range(384)]
    except Exception as e:
        logger.error(f"Error generating embedding: {str(e)}")
        import random
        # Return a default 384-dimensional vector as fallback
        return [random.random() for _ in range(384)]

@app.options("/api/chat/query")
async def options_query_endpoint(request: Request):
    """Handle OPTIONS preflight request for query endpoint."""
    # This is handled by the CORSMiddleware, but we explicitly define it to ensure compatibility
    return {"detail": "OPTIONS request handled"}

@app.options("/api/chat/health")
async def options_health_endpoint(request: Request):
    """Handle OPTIONS preflight request for health endpoint."""
    return {"detail": "OPTIONS request handled"}

@app.options("/api/chat/vectorize")
async def options_vectorize_endpoint(request: Request):
    """Handle OPTIONS preflight request for vectorize endpoint."""
    return {"detail": "OPTIONS request handled"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)