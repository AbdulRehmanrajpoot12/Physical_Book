# Quickstart Guide: Data Processing & Embedding Pipeline

## Prerequisites

- Python 3.8 or higher
- uv package manager
- Cohere API key
- Qdrant instance (local or cloud)

## Setup

### 1. Install uv (if not already installed)

```bash
pip install uv
```

### 2. Navigate to backend directory

```bash
cd backend
```

### 3. Install dependencies

```bash
uv pip install requests python-dotenv cohere qdrant-client
```

### 4. Set up environment variables

Create a `.env` file with your API keys:

```bash
# .env file
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # e.g., https://your-cluster.europe-west3-4.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here  # Optional if using local Qdrant
EXTERNAL_API_URL=your_external_api_url_here  # The API to fetch data from
```

### 5. Run the data processing pipeline

```bash
uv run main.py
```

## Configuration

The system can be customized by modifying these parameters in `main.py` or via environment variables:

- `EXTERNAL_API_URL`: The external API endpoint to fetch data from
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: "spec2_embedding")
- `RATE_LIMIT_DELAY`: Delay between API requests in seconds (default: 1.0)

## Architecture Overview

The system consists of these main functions:

1. `fetch_data()` - Fetches data from external API with rate limiting
2. `clean_data()` - Cleans and preprocesses raw data
3. `transform_data()` - Transforms cleaned data with calculations
4. `embed()` - Generates embeddings using Cohere
5. `create_collection()` - Creates Qdrant collection named "spec2_embedding"
6. `save_to_qdrant()` - Stores embeddings with metadata in Qdrant
7. `main()` - Orchestrates the entire data processing pipeline

## Troubleshooting

- If you get API key errors, verify your Cohere and Qdrant credentials in `.env`
- If data fetching fails, check that the external API URL is accessible
- For large datasets, monitor memory usage and adjust processing batch sizes
- If rate limiting is too aggressive, adjust the `RATE_LIMIT_DELAY` value