# Data Model: Content Ingestion System

## Entities

### Document
Represents a single chunk of content from the book with its embedding and metadata.

**Fields:**
- `id` (string): Unique identifier for the document chunk (MD5 hash of content + metadata)
- `content` (string): The actual text content of the chunk
- `url` (string): Source URL where the content was extracted from
- `title` (string): Page title extracted from the HTML
- `headings` (list of strings): List of headings found in the original document
- `embedding` (list of floats): Vector representation of the content (1024-dimensional Cohere embedding)
- `created_at` (float): Unix timestamp of when the document was processed

**Relationships:**
- One source page may result in multiple document chunks after text splitting

### Ingestion Configuration
Parameters that control the ingestion process.

**Fields:**
- `target_url` (string): Base URL to crawl (https://abdulrehmanrajpoot12.github.io/Physical_Book/)
- `chunk_size` (integer): Maximum size of text chunks (default: 500 characters)
- `chunk_overlap` (integer): Overlap between consecutive chunks (default: 50 characters)
- `rate_limit_delay` (float): Delay in seconds between requests (default: 1.0)

## Vector Database Schema (Qdrant)

### Collection: rag_embedding
A Qdrant collection to store document embeddings for semantic search.

**Vector Configuration:**
- `size`: 1024 (dimension of Cohere embeddings)
- `distance`: COSINE (cosine similarity for semantic search)

**Payload Fields:**
- `text` (string): The original text content
- `url` (string): Source URL
- `title` (string): Document title
- `headings` (list of strings): Document headings
- `created_at` (float): Unix timestamp

## Data Flow

1. **URL Discovery**: System discovers URLs from the base book URL (via sitemap or crawling)
2. **Content Extraction**: HTML content is extracted and cleaned from each URL
3. **Text Chunking**: Long documents are split into smaller chunks to fit embedding models
4. **Embedding Generation**: Each chunk is converted to a 1024-dimensional vector using Cohere
5. **Storage**: Embeddings with metadata are stored in Qdrant collection with the document text as payload