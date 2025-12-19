# Data Model: Data Processing & Embedding Pipeline

## Entities

### Raw Data
Represents unprocessed data fetched from external APIs that requires cleaning and transformation.

**Fields:**
- `id` (string): Unique identifier for the raw data record
- `source_api` (string): The API endpoint where data was fetched from
- `raw_content` (dict): The unprocessed data from the API
- `fetch_timestamp` (float): Unix timestamp when the data was fetched
- `status` (string): Processing status (e.g., "fetched", "cleaning", "cleaned", "error")

### Processed Data
Represents cleaned and transformed data ready for embedding generation.

**Fields:**
- `id` (string): Unique identifier (corresponds to the raw data ID)
- `clean_content` (dict): The cleaned and transformed data
- `metadata` (dict): Information about the processing (source, transformations applied, etc.)
- `transform_timestamp` (float): Unix timestamp when the data was transformed
- `validation_status` (string): Status of data validation (e.g., "valid", "invalid", "needs_review")

### Embedding
A vector representation of processed data that captures semantic meaning, stored with associated metadata.

**Fields:**
- `id` (string): Unique identifier (corresponds to the processed data ID)
- `embedding` (list of floats): Vector representation from Cohere (1024-dimensional for Cohere models)
- `source_data_id` (string): Reference to the source processed data
- `embedding_timestamp` (float): Unix timestamp when the embedding was generated
- `quality_score` (float): Quality assessment of the embedding (0.0-1.0)

### Qdrant Record
A stored entry in the Qdrant vector database containing embeddings and associated metadata.

**Fields:**
- `point_id` (string): Unique identifier in Qdrant (MD5 hash of content + metadata)
- `vector` (list of floats): The embedding vector
- `payload` (dict): Metadata including original content, source, timestamps
  - `content` (string): Original text content
  - `source_id` (string): ID of the source processed data
  - `source_api` (string): API where the original data was fetched from
  - `created_at` (float): Unix timestamp
- `collection_name` (string): Name of the Qdrant collection ("spec2_embedding")

## Data Flow

1. **Data Fetching**: System fetches raw data from external APIs
2. **Data Cleaning**: Raw data is validated and cleaned to remove inconsistencies
3. **Data Transformation**: Cleaned data undergoes required transformations and calculations
4. **Embedding Generation**: Processed data is converted to vector embeddings using Cohere
5. **Storage**: Embeddings with metadata are stored in Qdrant collection "spec2_embedding"