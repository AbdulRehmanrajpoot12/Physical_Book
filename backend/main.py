"""
Data Processing & Embedding Pipeline

This script implements a complete data processing pipeline that:
- Fetches data from external APIs
- Cleans and preprocesses the data
- Transforms the data with calculations
- Generates embeddings using Cohere
- Stores embeddings in Qdrant collection named 'spec2_embedding'
"""

import os
import requests
import logging
from typing import Dict, List, Any, Optional
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import time
import hashlib
import json
import traceback
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DataProcessingPipeline:
    """
    A class to manage the complete data processing pipeline:
    - Fetch data from external APIs
    - Clean and preprocess the data
    - Transform the data with calculations
    - Generate embeddings using Cohere
    - Store embeddings in Qdrant
    """

    def __init__(self):
        """
        Initialize the pipeline with configuration from environment variables.

        Raises:
            ValueError: If required environment variables are not set
        """
        # Initialize Cohere client
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        self.cohere_client = cohere.Client(cohere_api_key)

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if qdrant_api_key:
            self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url)

        # External API URL
        self.external_api_url = os.getenv("EXTERNAL_API_URL")
        if not self.external_api_url:
            raise ValueError("EXTERNAL_API_URL environment variable is required")

        # Rate limiting delay
        self.rate_limit_delay = float(os.getenv("RATE_LIMIT_DELAY", "1.0"))

        # Session with retry strategy for API calls
        self.session = requests.Session()
        retry_strategy = Retry(
            total=3,
            backoff_factor=1,
            status_forcelist=[429, 500, 502, 503, 504],
        )
        adapter = HTTPAdapter(max_retries=retry_strategy)
        self.session.mount("http://", adapter)
        self.session.mount("https://", adapter)

    def fetch_data(self) -> List[Dict[str, Any]]:
        """
        Fetch data from external API(s) with rate limiting and retry logic.

        Returns:
            List[Dict[str, Any]]: A list of records fetched from the external API

        Raises:
            requests.exceptions.RequestException: If there's an issue with the API request
            ValueError: If the API response format is unexpected
        """
        logger.info(f"Fetching data from external API: {self.external_api_url}")

        try:
            # Add rate limiting
            time.sleep(self.rate_limit_delay)

            response = self.session.get(self.external_api_url, timeout=30)
            response.raise_for_status()

            data = response.json()

            # Handle different response formats
            if isinstance(data, list):
                records = data
            elif isinstance(data, dict):
                # Common patterns for API responses
                if 'data' in data:
                    records = data['data']
                elif 'results' in data:
                    records = data['results']
                elif 'items' in data:
                    records = data['items']
                else:
                    # If it's a single object, wrap it in a list
                    records = [data]
            else:
                raise ValueError(f"Unexpected data format: {type(data)}")

            logger.info(f"Fetched {len(records)} records from external API")
            return records

        except requests.exceptions.RequestException as e:
            logger.error(f"Error fetching data from API: {e}")
            if hasattr(e, 'response') and e.response is not None:
                logger.error(f"API returned status code: {e.response.status_code}")
                logger.error(f"API response text: {e.response.text[:500]}...")  # First 500 chars
            raise
        except ValueError as e:
            logger.error(f"Error parsing API response: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error during data fetch: {e}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            raise

    def clean_data(self, raw_data: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Clean and preprocess the fetched data with validation and error handling.

        Args:
            raw_data: List of raw data records to clean

        Returns:
            List[Dict[str, Any]]: List of cleaned data records
        """
        logger.info(f"Cleaning {len(raw_data)} raw data records")

        cleaned_records = []

        for i, record in enumerate(raw_data):
            try:
                # Basic cleaning operations
                cleaned_record = {}

                for key, value in record.items():
                    # Skip None values
                    if value is None:
                        continue

                    # Clean string values
                    if isinstance(value, str):
                        cleaned_value = value.strip()
                        if cleaned_value:  # Only add non-empty strings
                            cleaned_record[key] = cleaned_value
                    else:
                        cleaned_record[key] = value

                # Add record ID if not present
                if 'id' not in cleaned_record:
                    cleaned_record['id'] = f"record_{i}_{int(time.time())}"

                # Add source information
                cleaned_record['source_api'] = self.external_api_url
                cleaned_record['fetch_timestamp'] = time.time()

                cleaned_records.append(cleaned_record)

            except Exception as e:
                logger.warning(f"Error cleaning record {i}: {e}. Skipping this record.")
                logger.debug(f"Problematic record {i}: {record}")
                continue

        logger.info(f"Cleaned data contains {len(cleaned_records)} records")
        return cleaned_records

    def transform_data(self, cleaned_data: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Perform required transformations or calculations on the cleaned data.

        Args:
            cleaned_data: List of cleaned data records to transform

        Returns:
            List[Dict[str, Any]]: List of transformed data records with calculated fields
        """
        logger.info(f"Transforming {len(cleaned_data)} cleaned data records")

        transformed_records = []

        for i, record in enumerate(cleaned_data):
            try:
                # Example transformations - these can be customized based on specific needs
                transformed_record = record.copy()

                # Add calculated fields
                transformed_record['processed_at'] = time.time()

                # If there's text content, add content length
                text_content = None
                for key in ['content', 'text', 'description', 'body']:
                    if key in transformed_record and isinstance(transformed_record[key], str):
                        text_content = transformed_record[key]
                        break

                if text_content:
                    transformed_record['content_length'] = len(text_content)
                    transformed_record['word_count'] = len(text_content.split())

                # Add hash for deduplication
                content_hash = hashlib.md5(
                    json.dumps(transformed_record, sort_keys=True).encode()
                ).hexdigest()
                transformed_record['content_hash'] = content_hash

                transformed_records.append(transformed_record)

            except Exception as e:
                logger.warning(f"Error transforming record {i}: {e}. Skipping this record.")
                logger.debug(f"Problematic record {i}: {record}")
                continue

        logger.info(f"Transformed data contains {len(transformed_records)} records")
        return transformed_records

    def embed(self, text: str) -> List[float]:
        """
        Generate embeddings using Cohere with error handling and validation.

        Args:
            text: The text to generate embeddings for

        Returns:
            List[float]: The embedding vector (1024-dimensional for Cohere models)

        Raises:
            Exception: If there's an error generating the embedding
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",  # Using Cohere's English embedding model
                input_type="search_document"  # Specify the input type
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating embedding for text: {e}")
            logger.debug(f"Text that failed: {text[:200]}...")  # First 200 chars
            raise

    def create_collection(self, collection_name: str):
        """
        Create Qdrant collection with proper error handling and validation.

        Args:
            collection_name: The name of the collection to create
        """
        try:
            # Check if collection already exists
            try:
                self.qdrant_client.get_collection(collection_name)
                logger.info(f"Collection {collection_name} already exists")
                return
            except:
                pass  # Collection doesn't exist, so create it

            # Create collection with vector configuration
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=1024,  # Cohere embeddings are 1024-dimensional for embed-english-v3.0
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection: {collection_name}")
        except Exception as e:
            logger.error(f"Error creating collection {collection_name}: {e}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            raise

    def save_to_qdrant(self, embedding: List[float], metadata: Dict[str, Any]):
        """
        Store embeddings/results in Qdrant with error handling and validation.

        Args:
            embedding: The embedding vector to store
            metadata: The metadata associated with the embedding

        Raises:
            Exception: If there's an error saving to Qdrant
        """
        try:
            # Generate a unique ID for this embedding
            content_hash = hashlib.md5(
                (str(embedding) + json.dumps(metadata, sort_keys=True)).encode()
            ).hexdigest()

            # Prepare the payload
            self.qdrant_client.upsert(
                collection_name="spec2_embedding",
                points=[
                    models.PointStruct(
                        id=content_hash,
                        vector=embedding,
                        payload=metadata
                    )
                ]
            )
            logger.info(f"Saved embedding to Qdrant with ID: {content_hash[:8]}...")
        except Exception as e:
            logger.error(f"Error saving to Qdrant: {e}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            logger.debug(f"Embedding length: {len(embedding) if embedding else 'None'}")
            logger.debug(f"Metadata keys: {list(metadata.keys()) if metadata else 'None'}")
            raise

def main():
    """
    Execute all steps in the main function with comprehensive error handling.

    This function orchestrates the entire data processing pipeline:
    1. Initializes the pipeline
    2. Creates the Qdrant collection
    3. Fetches data from external API
    4. Cleans and transforms the data
    5. Generates embeddings and stores them in Qdrant
    """
    logger.info("Starting data processing pipeline")
    start_time = time.time()

    try:
        # Initialize the pipeline
        pipeline = DataProcessingPipeline()

        # Create the collection
        collection_name = "spec2_embedding"
        pipeline.create_collection(collection_name)

        # Fetch data from external API
        raw_data = pipeline.fetch_data()
        logger.info(f"Fetched {len(raw_data)} records from external API")

        # Clean the data
        cleaned_data = pipeline.clean_data(raw_data)
        logger.info(f"Cleaned {len(cleaned_data)} records")

        # Transform the data
        transformed_data = pipeline.transform_data(cleaned_data)
        logger.info(f"Transformed {len(transformed_data)} records")

        # Process each transformed record
        successful_records = 0
        failed_records = 0

        for i, record in enumerate(transformed_data):
            logger.info(f"Processing record {i+1}/{len(transformed_data)}")

            try:
                # Convert record to text for embedding
                # Prioritize content fields, fall back to the entire record as JSON
                text_content = ""
                for field in ['content', 'text', 'description', 'body', 'title']:
                    if field in record and isinstance(record[field], str):
                        text_content = record[field]
                        break

                if not text_content:
                    # If no specific content field, convert the entire record to string
                    text_content = json.dumps({k: v for k, v in record.items()
                                             if isinstance(v, (str, int, float, bool))},
                                            ensure_ascii=False)

                # Generate embedding
                embedding = pipeline.embed(text_content)

                # Prepare metadata
                metadata = {
                    **record,  # Include all original record fields
                    "embedding_generated_at": time.time(),
                    "source_record_index": i
                }

                # Save to Qdrant
                pipeline.save_to_qdrant(embedding, metadata)
                successful_records += 1

            except Exception as record_error:
                logger.error(f"Failed to process record {i}: {record_error}")
                logger.debug(f"Failed record {i}: {record}")
                failed_records += 1
                continue  # Continue with next record

        total_time = time.time() - start_time
        logger.info(f"Data processing pipeline completed successfully")
        logger.info(f"Processed {len(transformed_data)} total records: "
                   f"{successful_records} successful, {failed_records} failed, "
                   f"in {total_time:.2f} seconds")

    except Exception as e:
        logger.error(f"Data processing pipeline failed: {e}")
        logger.error(f"Traceback: {traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()