# RAG Agent with Retrieval Capabilities

This project implements a RAG (Retrieval Augmented Generation) agent that allows users to query book content and receive AI-generated answers based on relevant information retrieved from a vector database.

## Features

- Query book content using natural language
- Retrieve relevant contexts from Qdrant vector database
- Generate contextual responses using OpenAI
- FastAPI-based REST API for easy integration
- Configurable parameters for retrieval and response generation
- Health check and monitoring endpoints

## Prerequisites

- Python 3.8 or higher
- uv package manager
- OpenAI API key
- Qdrant instance (local or cloud)
- Cohere API key (for query embeddings)

## Setup

1. Install uv (if not already installed):
   ```bash
   pip install uv
   ```

2. Navigate to the rag_agent directory:
   ```bash
   cd backend/rag_agent
   ```

3. Install dependencies:
   ```bash
   uv pip install fastapi uvicorn openai qdrant-client cohere python-dotenv pydantic pydantic-settings
   ```

4. Set up environment variables by copying the example:
   ```bash
   cp .env.example .env
   ```

5. Edit the `.env` file with your API keys and configuration:
   ```bash
   # OpenAI API key for generating responses
   OPENAI_API_KEY=your_openai_api_key_here

   # Qdrant configuration
   QDRANT_URL=your_qdrant_url_here  # e.g., https://your-cluster.europe-west3-4.gcp.cloud.qdrant.io:6333
   QDRANT_API_KEY=your_qdrant_api_key_here  # Optional if using local Qdrant

   # Cohere API key for generating embeddings for queries
   COHERE_API_KEY=your_cohere_api_key_here
   ```

## Usage

Run the RAG agent API:

```bash
uv run main.py
```

Or with uvicorn:
```bash
uvicorn rag_agent.main:app --reload --host 0.0.0.0 --port 8000
```

## API Endpoints

- `GET /` - Root endpoint to verify the service is running
- `POST /query` - Query the book content (request body: {"query": "your question", "top_k": 5})
- `GET /health` - Health check endpoint
- `POST /config` - Update agent configuration
- `GET /config` - Get current agent configuration

## Example Query

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key concepts in this book?",
    "top_k": 5
  }'
```

## Configuration

The agent can be configured via the `/config` endpoint or by modifying the default values in the code:
- `retrieval_depth`: Number of results to retrieve from the vector database
- `response_style`: Style of the generated response (e.g., "concise", "detailed")
- `temperature`: OpenAI temperature parameter for response generation

## Architecture

The system consists of these main components:

1. `retrieve_contexts()` - Retrieves relevant content from Qdrant based on query embeddings
2. `generate_response_with_context()` - Generates AI responses using OpenAI with retrieved context
3. FastAPI endpoints for API access
4. Configuration management for agent parameters

## Integration with Book Project

The RAG agent is designed to be easily integrated into book projects via its REST API. Frontend applications can make HTTP requests to the API endpoints to query book content and receive AI-generated answers.

## Troubleshooting

- If you get API key errors, verify your OpenAI, Qdrant, and Cohere credentials in `.env`
- If retrieval fails, check that the Qdrant collection "spec2_embedding" exists and contains data
- For slow responses, consider reducing the `top_k` parameter or optimizing your vector database
- Check that your book content has been properly indexed in Qdrant through the data pipeline