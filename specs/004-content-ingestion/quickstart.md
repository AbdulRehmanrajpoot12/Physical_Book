# Quickstart Guide: Content Ingestion System

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

### 2. Clone the repository and navigate to backend directory

```bash
cd backend
```

### 3. Create virtual environment and install dependencies

```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
```

### 4. Set up environment variables

Create a `.env` file with your API keys:

```bash
# .env file
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # e.g., https://your-cluster.europe-west3-4.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here  # Optional if using local Qdrant
```

### 5. Run the ingestion system

```bash
python main.py
```

## Configuration

The system can be customized by modifying these parameters in `main.py`:

- `chunk_size`: Maximum size of text chunks (default: 500)
- `chunk_overlap`: Overlap between consecutive chunks (default: 50)
- `rate_limit_delay`: Delay between requests in seconds (default: 1.0)
- `collection_name`: Name of the Qdrant collection (default: "rag_embedding")

## Architecture Overview

The system consists of these main functions:

1. `get_all_urls()` - Discovers all URLs from the target site via sitemap or crawling
2. `extract_text_from_url()` - Extracts clean text content, title, and headings from a URL
3. `chunk_text()` - Splits long documents into manageable chunks
4. `embed()` - Generates embeddings using Cohere
5. `create_collection()` - Sets up Qdrant collection named "rag_embedding"
6. `save_chunk_to_qdrant()` - Stores embeddings with metadata in Qdrant
7. `main()` - Orchestrates the entire ingestion pipeline

## Troubleshooting

- If you get API key errors, verify your Cohere and Qdrant credentials in `.env`
- If URLs aren't being discovered, check that the target URL is accessible
- For large documents, adjust `chunk_size` and `chunk_overlap` parameters
- If rate limiting is too aggressive, reduce the `rate_limit_delay` value