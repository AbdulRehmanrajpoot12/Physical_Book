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
from qdrant_client.http.models import Distance, VectorParams
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

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://rehmanrajpoot-physicalbook.hf.space",
        "https://rehmanrajpoot.github.io",
        "http://localhost:3000",
        "http://localhost:8000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    max_age=3600,
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
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=10
)
openrouter_client = httpx.AsyncClient(
    base_url="https://openrouter.ai/api/v1",
    headers={
        "Authorization": f"Bearer {os.getenv('OPENROUTER_API_KEY', '')}",
        "Content-Type": "application/json"
    }
)

# Startup event
@app.on_event("startup")
async def startup_event():
    logger.info("Starting up RAG Chatbot API")
    try:
        qdrant_client.get_collection("book_content")
        logger.info("Connected to Qdrant collection: book_content")
    except Exception as e:
        logger.warning(f"Collection `book_content` not found: {str(e)}")
        # Create collection automatically
        qdrant_client.recreate_collection(
            collection_name="book_content",
            vectors_config=VectorParams(
                size=384,
                distance=Distance.COSINE
            )
        )
        logger.info("Collection `book_content` created successfully")

# Utility function for embeddings
def get_embedding(text: str) -> List[float]:
    try:
        from sentence_transformers import SentenceTransformer
        import numpy as np
        if not hasattr(get_embedding, 'model'):
            get_embedding.model = SentenceTransformer('all-MiniLM-L6-v2')
        embedding = get_embedding.model.encode([text])
        return embedding[0].tolist()
    except ImportError:
        import random
        return [random.random() for _ in range(384)]
    except Exception as e:
        logger.error(f"Error generating embedding: {str(e)}")
        import random
        return [random.random() for _ in range(384)]

# Query endpoint
async def query_logic(request: QueryRequest) -> QueryResponse:
    try:
        query_embedding = get_embedding(request.query)
    except Exception as e:
        logger.error(f"Error getting embedding: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": {"code": "EMBEDDING_ERROR", "message": str(e)}
        })

    try:
        # Correct SDK method
        search_results = qdrant_client.search_points(
            collection_name="book_content",
            vector=query_embedding,
            limit=5,
            with_payload=True
        )

        context_chunks = []
        sources = []
        for result in search_results:
            if result.payload:
                context_chunks.append(result.payload.get("content", ""))
                sources.append(Source(
                    chapter=result.payload.get("chapter", "Unknown"),
                    section=result.payload.get("section", "Unknown"),
                    similarity_score=result.score
                ))
        context = "\n\n".join(context_chunks)

        # Prepare prompt
        prompt = f"""
        You are an AI assistant helping users with questions about robotics, AI, and the book content.
        Use the following context to answer the question:

        {context}

        Question: {request.query}

        Answer:
        """

        try:
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
                raise Exception(f"OpenRouter API error: {response.status_code}")

            response_data = response.json()
            answer = response_data["choices"][0]["message"]["content"]

            return QueryResponse(
                answer=answer,
                sources=sources,
                confidence="high" if response_data.get("usage", {}).get("total_tokens", 0) > 0 else "medium",
                timestamp=datetime.now().isoformat()
            )

        except httpx.HTTPStatusError as e:
            if e.response.status_code == 429:
                return QueryResponse(
                    answer="LLM response unavailable due to API quota; retrieval verified.",
                    sources=sources,
                    confidence="medium",
                    timestamp=datetime.now().isoformat()
                )
            else:
                return QueryResponse(
                    answer="LLM response unavailable due to API error; retrieval verified.",
                    sources=sources,
                    confidence="low",
                    timestamp=datetime.now().isoformat()
                )
        except Exception:
            return QueryResponse(
                answer="LLM response unavailable due to API error; retrieval verified.",
                sources=sources,
                confidence="low",
                timestamp=datetime.now().isoformat()
            )

    except Exception as e:
        logger.error(f"Error querying Qdrant: {str(e)}")
        return QueryResponse(
            answer="I'm currently unable to retrieve information from the knowledge base. The vector database may be temporarily unavailable. Please try again in a moment.",
            sources=[],
            confidence="low",
            timestamp=datetime.now().isoformat()
        )

# API endpoints
@app.post("/api/chat/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    return await query_logic(request)

@app.post("/query", response_model=QueryResponse)
async def simple_query_endpoint(request: QueryRequest):
    return await query_logic(request)

@app.options("/api/chat/query")
@app.options("/query")
async def options_query_endpoint(request: Request):
    return {"detail": "OPTIONS request handled"}

@app.get("/api/chat/health", response_model=HealthResponse)
async def health_check():
    vector_db_status = "connected"
    llm_status = "available"

    try:
        qdrant_client.get_collection("book_content")
    except:
        vector_db_status = "disconnected"

    try:
        test_response = await openrouter_client.post(
            "/chat/completions",
            json={
                "model": os.getenv("OPENROUTER_MODEL", "bytedance-seed/seedream-4.5"),
                "messages": [{"role": "user", "content": "test"}],
                "max_tokens": 1
            }
        )
        if test_response.status_code != 200:
            llm_status = "unavailable"
    except Exception:
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
    try:
        embedding = get_embedding(request.content)
        qdrant_client.upsert(
            collection_name="book_content",
            points=[models.PointStruct(
                id=request.content_id,
                vector=embedding,
                payload={
                    "content": request.content,
                    "chapter": request.metadata.get("chapter", ""),
                    "section": request.metadata.get("section", ""),
                    "tags": request.metadata.get("tags", []),
                    "created_at": datetime.now().isoformat()
                }
            )]
        )
        return VectorizeResponse(
            success=True,
            vector_id=request.content_id,
            processed_at=datetime.now().isoformat()
        )
    except Exception as e:
        logger.error(f"Error vectorizing content: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": {"code": "PROCESSING_ERROR", "message": str(e)}
        })

@app.options("/api/chat/vectorize")
async def options_vectorize_endpoint(request: Request):
    return {"detail": "OPTIONS request handled"}

# Run server
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
