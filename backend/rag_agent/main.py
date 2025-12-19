"""
RAG Agent with Retrieval Capabilities

This script implements a RAG (Retrieval Augmented Generation) agent using OpenAI Agents SDK and FastAPI.
It integrates with Qdrant for vector search capabilities to retrieve relevant book content.
"""

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import os
import logging
import asyncio
from contextlib import asynccontextmanager
import openai
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere  # We'll use this for embeddings since we already have a data pipeline using Cohere
import uuid
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def validate_environment_variables():
    """Validate that all required environment variables are set."""
    required_vars = ["OPENAI_API_KEY", "QDRANT_URL", "QDRANT_API_KEY", "COHERE_API_KEY"]
    missing_vars = []

    for var in required_vars:
        value = os.getenv(var)
        if not value or value.strip() == "":
            missing_vars.append(var)

    if missing_vars:
        error_msg = f"Missing required environment variables: {', '.join(missing_vars)}"
        logger.error(error_msg)
        raise ValueError(error_msg)

    logger.info("All required environment variables are present")

def validate_qdrant_connection(qdrant_client: QdrantClient):
    """Validate Qdrant connection and check if the required collection exists."""
    try:
        # Test Qdrant connection by getting collections
        collections = qdrant_client.get_collections()
        logger.info("Connected to Qdrant successfully")

        # Check if the required collection exists
        collection_name = "spec2_embedding"
        collection_exists = False
        for collection in collections.collections:
            if collection.name == collection_name:
                collection_exists = True
                logger.info(f"Found required collection: {collection_name}")
                break

        if not collection_exists:
            logger.warning(f"Required collection '{collection_name}' not found in Qdrant")
            # We don't fail here as the collection might be created later, but we log the warning
            # In a production scenario, you might want to create it or fail fast
        else:
            # Get collection info to confirm it's accessible
            collection_info = qdrant_client.get_collection(collection_name)
            logger.info(f"Collection '{collection_name}' is accessible with {collection_info.points_count} points")

        return True
    except Exception as e:
        error_msg = f"Failed to connect to Qdrant or validate collection: {e}"
        logger.error(error_msg)
        raise ConnectionError(error_msg)

def validate_openai_connection(openai_client):
    """Validate OpenAI connection."""
    try:
        # Test OpenAI connection by making a simple call
        models = openai_client.models.list()
        logger.info("Connected to OpenAI successfully")
        return True
    except Exception as e:
        error_msg = f"Failed to connect to OpenAI: {e}"
        logger.error(error_msg)
        raise ConnectionError(error_msg)

def validate_cohere_connection():
    """Validate Cohere connection."""
    try:
        # Initialize Cohere client and test connection
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")

        cohere_client = cohere.Client(cohere_api_key)
        # Test with a simple embed call
        response = cohere_client.embed(
            texts=["test"],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        logger.info("Connected to Cohere successfully")
        return True
    except Exception as e:
        error_msg = f"Failed to connect to Cohere: {e}"
        logger.error(error_msg)
        raise ConnectionError(error_msg)

# Initialize FastAPI app
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Initializing RAG Agent services...")

    # Validate environment variables first
    validate_environment_variables()

    # Initialize clients during startup
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    try:
        app.state.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )
    except Exception as e:
        error_msg = f"Failed to initialize Qdrant client: {e}"
        logger.error(error_msg)
        raise ConnectionError(error_msg)

    try:
        app.state.openai_client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    except Exception as e:
        error_msg = f"Failed to initialize OpenAI client: {e}"
        logger.error(error_msg)
        raise ConnectionError(error_msg)

    # Verify all connections
    try:
        validate_qdrant_connection(app.state.qdrant_client)
        validate_openai_connection(app.state.openai_client)
        validate_cohere_connection()

        logger.info("All services connected and validated successfully")
    except Exception as e:
        logger.error(f"Service validation failed: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down RAG Agent services...")

app = FastAPI(
    title="RAG Agent API",
    description="API for RAG Agent with Retrieval Capabilities",
    version="0.1.0",
    lifespan=lifespan
)

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend origin
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods including OPTIONS
    allow_headers=["*"],  # Allow all headers
)

# Request/Response Models
class QueryRequest(BaseModel):
    query: str
    top_k: int = Query(default=5, ge=1, le=20, description="Number of results to retrieve")
    include_metadata: bool = Query(default=True, description="Include metadata in results")

class RetrievedContext(BaseModel):
    id: str
    content: str
    score: float
    metadata: Dict[str, Any]

class AgentResponse(BaseModel):
    query: str
    answer: str
    retrieved_contexts: List[RetrievedContext]
    sources: List[str]

class AgentConfig(BaseModel):
    retrieval_depth: int = 5
    response_style: str = "concise"
    temperature: float = 0.7

# Configuration
DEFAULT_AGENT_CONFIG = AgentConfig()

@app.get("/")
async def root():
    return {"message": "RAG Agent API is running"}

@app.post("/query", response_model=AgentResponse)
async def query_book_content(request: QueryRequest) -> AgentResponse:
    """
    Query the book content using the RAG agent.

    This endpoint accepts a user query, retrieves relevant context from Qdrant,
    and generates an AI response using OpenAI based on the retrieved context.
    """
    try:
        logger.info(f"Processing query: {request.query}")

        # Retrieve relevant contexts from Qdrant
        retrieved_contexts = await retrieve_contexts(
            request.query,
            request.top_k,
            app.state.qdrant_client
        )

        if not retrieved_contexts:
            logger.warning("No relevant contexts found for query")
            return AgentResponse(
                query=request.query,
                answer="I couldn't find any relevant content in the book to answer your question.",
                retrieved_contexts=[],
                sources=[]
            )

        # Generate response using OpenAI with retrieved context
        answer = await generate_response_with_context(
            request.query,
            retrieved_contexts,
            app.state.openai_client
        )

        # Extract sources from retrieved contexts
        sources = list(set([ctx.metadata.get('source_api', 'Unknown') for ctx in retrieved_contexts]))

        response = AgentResponse(
            query=request.query,
            answer=answer,
            retrieved_contexts=retrieved_contexts,
            sources=sources
        )

        logger.info(f"Successfully processed query, retrieved {len(retrieved_contexts)} contexts")
        return response

    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.get("/health")
async def health_check():
    """Health check endpoint to verify the service is running."""
    return {"status": "healthy", "service": "RAG Agent API"}

@app.post("/config")
async def update_agent_config(config: AgentConfig):
    """Update the agent configuration parameters."""
    global DEFAULT_AGENT_CONFIG
    DEFAULT_AGENT_CONFIG = config
    logger.info(f"Agent configuration updated: {config}")
    return {"message": "Configuration updated successfully", "config": config}

@app.get("/config")
async def get_agent_config() -> AgentConfig:
    """Get the current agent configuration."""
    return DEFAULT_AGENT_CONFIG

async def retrieve_contexts(query: str, top_k: int, qdrant_client: QdrantClient) -> List[RetrievedContext]:
    """
    Retrieve relevant contexts from Qdrant based on the query.

    Uses Cohere embeddings to convert the query to a vector and performs
    semantic search in the Qdrant collection.
    """
    try:
        # Initialize Cohere client for query embedding
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")

        cohere_client = cohere.Client(cohere_api_key)

        # Generate embedding for the query
        response = cohere_client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]

        # Search in Qdrant for similar embeddings
        search_results = qdrant_client.query_points(
            collection_name="spec2_embedding",  # Using the same collection from data pipeline
            query=query_embedding,
            limit=top_k,
            with_payload=True
        )

        # Convert search results to RetrievedContext objects
        contexts = []
        for result in search_results.points:
            content = ""
            # Extract content from payload - try different possible content fields
            for field in ['content', 'text', 'description', 'body', 'title']:
                if field in result.payload:
                    content = result.payload[field]
                    break
            # If no specific content field found, convert entire payload to string
            if not content:
                content = str(result.payload)

            # Note: The 'request' variable is not available in this function's scope
            # We'll always include metadata for now, but this could be made configurable
            context = RetrievedContext(
                id=str(result.id),
                content=content,
                score=result.score,
                metadata=result.payload  # Always include metadata for now
            )
            contexts.append(context)

        logger.info(f"Retrieved {len(contexts)} contexts for query")
        return contexts

    except Exception as e:
        logger.error(f"Error retrieving contexts: {e}")
        raise

async def generate_response_with_context(
    query: str,
    contexts: List[RetrievedContext],
    openai_client: openai.OpenAI
) -> str:
    """
    Generate a response using OpenAI based on the query and retrieved contexts.
    """
    try:
        # Validate OpenAI API key is available
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("OPENAI_API_KEY environment variable is not set")

        # Combine contexts into a single context string
        context_str = "\n\n".join([
            f"Source {i+1}: {ctx.content[:500]}..."  # Limit context length
            for i, ctx in enumerate(contexts)
        ])

        # Create a prompt for OpenAI
        prompt = f"""
        You are an AI assistant helping users with questions about book content.
        Use the following retrieved contexts to answer the question.
        If the contexts don't contain enough information to answer the question,
        clearly state that you couldn't find relevant information.

        Retrieved Contexts:
        {context_str}

        Question: {query}

        Answer concisely based on the provided contexts, and cite sources when possible:
        """

        # Call OpenAI API to generate response
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",  # Can be configured
            messages=[
                {"role": "system", "content": "You are a helpful assistant that answers questions based on provided contexts. Be concise and cite sources when possible."},
                {"role": "user", "content": prompt}
            ],
            temperature=DEFAULT_AGENT_CONFIG.temperature,
            max_tokens=500
        )

        answer = response.choices[0].message.content
        logger.info(f"Generated response of length {len(answer)} characters")
        return answer

    except openai.RateLimitError as e:
        # Handle OpenAI 429 rate limit / quota exceeded errors gracefully
        logger.warning(f"OpenAI rate limit exceeded: {e}")
        return "LLM response unavailable due to API quota; retrieval verified. The system successfully retrieved relevant content from the database, but the language model is currently unavailable due to API quota limits."
    except Exception as e:
        logger.error(f"Error generating response: {e}")
        raise

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)