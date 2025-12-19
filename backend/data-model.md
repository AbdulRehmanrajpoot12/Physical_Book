# Data Model: Content Ingestion System

## Entities

### Document
Represents a single page or section from the Docusaurus book with its content and metadata.

**Fields:**
- `id` (string): Unique identifier for the document/chunk (MD5 hash of content + metadata)
- `content` (string): The actual text content of the document
- `url` (string): Source URL where the content was extracted from
- `title` (string): Page title extracted from the HTML
- `headings` (list of strings): List of headings found in the document
- `embedding` (list of floats): Vector representation of the content (1024-dimensional for Cohere embeddings)
- `created_at` (float): Unix timestamp of when the document was processed
- `chunk_index` (integer): Index of this chunk if the document was split
- `total_chunks` (integer): Total number of chunks this document was split into

**Relationships:**
- One document may result in multiple document chunks after text splitting

### Ingestion Configuration
Parameters that control the ingestion process.

**Fields:**
- `target_urls` (list of strings): URLs to crawl and extract content from
- `content_selectors` (list of strings): CSS selectors to identify content areas in HTML
- `chunk_size` (integer): Maximum size of text chunks (default: 500 characters)
- `chunk_overlap` (integer): Overlap between consecutive chunks (default: 50 characters)

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