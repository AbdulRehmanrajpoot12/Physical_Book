# Implementation Plan: Content Ingestion & Vector Storage

**Feature**: 004-content-ingestion
**Created**: 2025-12-19
**Status**: Draft
**Input**: User requirements for backend implementation with uv, URL discovery, text extraction/chunking, Cohere embeddings, and Qdrant storage

## Technical Context

The system will be implemented as a Python backend that ingests content from the deployed Docusaurus book at https://abdulrehmanrajpoot12.github.io/Physical_Book/, processes the content, generates embeddings using Cohere, and stores them in Qdrant vector database. The implementation will be contained in a single `main.py` file with specific required functions.

**Technology Stack:**
- Python 3.8+
- uv (for project management)
- Requests/BeautifulSoup (for web scraping)
- Cohere (for embeddings)
- Qdrant (vector database)
- python-dotenv (for environment management)

Rate limiting will be handled by implementing a 1-second delay between requests to be respectful to the target server.
The system is designed to handle books of moderate size (up to 1000 pages) with conservative memory management for large-scale processing.

## Constitution Check

Based on the project constitution, this implementation must:
- Follow spec-first development (requirements already specified)
- Maintain accuracy and faithfulness to source content
- Ensure reproducibility of the data pipeline
- Use Qdrant Cloud as specified in constitution (line 42)
- Avoid client-side API keys (for Cohere and Qdrant)
- Follow deterministic chunking preserving content structure

**Potential Violations:**
- None detected - all requirements align with project constitution

## Gates Evaluation

✅ **No violations detected** - all requirements align with project constitution.

## Phase 0: Outline & Research

### Research Tasks

1. **Decision: Rate Limiting Strategy**
   - Rationale: Need to respect the target website's resources during crawling
   - Alternative considered: No rate limiting vs. conservative rate limiting
   - Decision: Implement 1 request per second to be respectful

2. **Decision: Content Extraction Strategy**
   - Rationale: Need to extract meaningful content from Docusaurus pages
   - Alternative considered: Generic HTML text extraction vs. Docusaurus-specific selectors
   - Decision: Use common Docusaurus content selectors with fallback to generic extraction

### Research Findings

- Qdrant is a vector database that supports semantic search and is available as a cloud service
- Cohere provides high-quality text embeddings via API with the embed-english-v3.0 model
- uv is a fast Python package installer and resolver written in Rust
- The target deployment URL is https://abdulrehmanrajpoot12.github.io/Physical_Book/
- Docusaurus sites typically have structured content in main/article tags or specific CSS classes

## Phase 1: Design & Contracts

### Data Model

**Document Entity:**
- id: string (unique identifier)
- content: string (the text content)
- url: string (source URL)
- title: string (page title)
- headings: list of strings (headings in the content)
- embedding: list of floats (vector representation)
- created_at: float (timestamp)

**Ingestion Configuration:**
- target_urls: list of strings (URLs to crawl)
- chunk_size: integer (size of text chunks, default: 500)
- chunk_overlap: integer (overlap between chunks, default: 50)

### API Contract

Since this is a backend script, the contract is defined by the required functions:

1. `get_all_urls(base_url)` - discovers all URLs from the target site
2. `extract_text_from_url(url)` - extracts text content, title, and headings from a URL
3. `chunk_text(text, chunk_size=500, chunk_overlap=50)` - splits text into chunks
4. `embed(text)` - generates embeddings for text using Cohere
5. `create_collection(collection_name)` - creates a Qdrant collection named "rag_embedding"
6. `save_chunk_to_qdrant(chunk, embedding, metadata)` - saves chunk to Qdrant with metadata

### Quickstart Guide

1. Install uv: `pip install uv`
2. Clone the repository
3. Navigate to the backend directory
4. Run `uv venv` to create virtual environment
5. Run `uv pip install` to install dependencies
6. Set environment variables for Cohere and Qdrant
7. Run `python main.py` to execute the ingestion pipeline

## Phase 1: Implementation Plan

### Directory Structure
```
backend/
├── pyproject.toml (uv project file)
├── main.py (main ingestion script with all required functions)
├── .env.example
└── README.md
```

### Implementation Steps

1. Create backend directory and initialize with uv
2. Install required dependencies (requests, beautifulsoup4, cohere, qdrant-client, python-dotenv)
3. Implement URL discovery function with sitemap and crawling fallback
4. Implement text extraction with Docusaurus-specific selectors
5. Implement text chunking with sentence-boundary awareness
6. Implement embedding function using Cohere
7. Implement Qdrant collection creation for "rag_embedding"
8. Implement chunk saving to Qdrant
9. Create main execution function that orchestrates the entire pipeline
10. Add error handling, logging, and rate limiting