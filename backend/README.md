# Backend Services

This directory contains multiple backend services for the AI Physical Book project:

## Data Processing & Embedding Pipeline

This project implements a complete data processing pipeline that fetches data from external APIs, cleans and transforms it, generates embeddings using Cohere, and stores them in Qdrant for semantic search capabilities.

### Features

- Fetch data from external APIs with rate limiting and retry logic
- Clean and preprocess data with validation
- Transform data with calculated fields and metadata
- Generate embeddings using Cohere
- Store embeddings in Qdrant vector database
- Comprehensive error handling and logging
- Performance monitoring and metrics

## RAG Agent with Retrieval Capabilities

This project implements a RAG (Retrieval Augmented Generation) agent that allows users to query book content and receive AI-generated answers based on relevant information retrieved from a vector database.

### Features

- Query book content using natural language
- Retrieve relevant contexts from Qdrant vector database
- Generate contextual responses using OpenAI
- FastAPI-based REST API for easy integration
- Configurable parameters for retrieval and response generation
- Health check and monitoring endpoints

## Data Processing Pipeline Prerequisites

- Python 3.8 or higher
- uv package manager
- Cohere API key
- Qdrant instance (local or cloud)

## RAG Agent Prerequisites

- Python 3.8 or higher
- uv package manager
- OpenAI API key
- Qdrant instance (local or cloud)
- Cohere API key (for query embeddings)

## Data Processing Pipeline Setup

1. Install uv (if not already installed):
   ```bash
   pip install uv
   ```

2. Navigate to the backend directory:
   ```bash
   cd backend
   ```

3. Install dependencies:
   ```bash
   uv pip install requests python-dotenv cohere qdrant-client
   ```

4. Set up environment variables by copying the example:
   ```bash
   cd backend  # Navigate to the backend directory
   cp .env.example .env
   ```

5. Edit the `.env` file with your API keys and configuration:
   ```bash
   # Cohere API key for generating embeddings
   COHERE_API_KEY=your_cohere_api_key_here

   # Qdrant configuration
   QDRANT_URL=your_qdrant_url_here  # e.g., https://your-cluster.europe-west3-4.gcp.cloud.qdrant.io:6333
   QDRANT_API_KEY=your_qdrant_api_key_here  # Optional if using local Qdrant

   # External API configuration
   EXTERNAL_API_URL=your_external_api_url_here  # The API to fetch data from

   # Rate limiting configuration
   RATE_LIMIT_DELAY=1.0  # Delay in seconds between API requests
   ```

## RAG Agent Setup

1. Navigate to the rag_agent directory:
   ```bash
   cd backend/rag_agent
   ```

2. Install dependencies:
   ```bash
   uv pip install fastapi uvicorn openai qdrant-client cohere python-dotenv pydantic pydantic-settings
   ```

3. Set up environment variables by copying the example:
   ```bash
   cp .env.example .env
   ```

4. Edit the `.env` file with your API keys and configuration:
   ```bash
   # OpenAI API key for generating responses
   OPENAI_API_KEY=your_openai_api_key_here

   # Qdrant configuration
   QDRANT_URL=your_qdrant_url_here  # e.g., https://your-cluster.europe-west3-4.gcp.cloud.qdrant.io:6333
   QDRANT_API_KEY=your_qdrant_api_key_here  # Optional if using local Qdrant

   # Cohere API key for generating embeddings for queries
   COHERE_API_KEY=your_cohere_api_key_here
   ```

## Data Processing Pipeline Usage

Run the data processing pipeline:

```bash
uv run main.py
```

## RAG Agent Usage

Run the RAG agent API:

```bash
cd rag_agent
uv run main.py
```

Or with uvicorn:
```bash
cd rag_agent
uvicorn rag_agent.main:app --reload --host 0.0.0.0 --port 8000
```

## Data Processing Pipeline Configuration

The system can be customized by modifying these parameters in `main.py` or via environment variables:

- `EXTERNAL_API_URL`: The external API endpoint to fetch data from
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: "spec2_embedding")
- `RATE_LIMIT_DELAY`: Delay between API requests in seconds (default: 1.0)

## RAG Agent Configuration

The RAG agent can be customized via API endpoints or environment variables:

- `OPENAI_API_KEY`: OpenAI API key for generating responses
- `QDRANT_URL`: URL for the Qdrant vector database
- `COHERE_API_KEY`: Cohere API key for generating query embeddings
- Configuration parameters can also be updated via the `/config` API endpoint

## Data Processing Pipeline Architecture

The system consists of these main components:

1. `fetch_data()` - Fetches data from external API with rate limiting
2. `clean_data()` - Cleans and preprocesses raw data
3. `transform_data()` - Transforms cleaned data with calculations
4. `embed()` - Generates embeddings using Cohere
5. `create_collection()` - Creates Qdrant collection named "spec2_embedding"
6. `save_to_qdrant()` - Stores embeddings with metadata in Qdrant
7. `main()` - Orchestrates the entire data processing pipeline

## RAG Agent Architecture

The RAG agent system consists of these main components:

1. FastAPI endpoints for API access
2. `retrieve_contexts()` - Retrieves relevant content from Qdrant based on query embeddings
3. `generate_response_with_context()` - Generates AI responses using OpenAI with retrieved context
4. Configuration management for agent parameters
5. Health check and monitoring endpoints

## Troubleshooting

### Data Processing Pipeline
- If you get API key errors, verify your Cohere and Qdrant credentials in `.env`
- If data fetching fails, check that the external API URL is accessible
- For large datasets, monitor memory usage and adjust processing batch sizes
- If rate limiting is too aggressive, adjust the `RATE_LIMIT_DELAY` value

### RAG Agent
- If you get API key errors, verify your OpenAI, Qdrant, and Cohere credentials in `.env`
- If retrieval fails, check that the Qdrant collection "spec2_embedding" exists and contains data
- For slow responses, consider reducing the `top_k` parameter or optimizing your vector database
- Check that your book content has been properly indexed in Qdrant through the data pipeline