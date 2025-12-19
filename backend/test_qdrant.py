"""
Test script to verify Qdrant connection and check if any embeddings were stored
"""

from qdrant_client import QdrantClient
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_api_key:
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
else:
    client = QdrantClient(url=qdrant_url)

# Check if the collection exists
try:
    collection_info = client.get_collection("rag_embedding")
    print(f"Collection 'rag_embedding' exists")
    print(f"Collection vectors count: {collection_info.points_count}")

    # Try to get a few points if any exist
    if collection_info.points_count > 0:
        points = client.scroll(
            collection_name="rag_embedding",
            limit=2
        )
        print(f"Sample points retrieved: {len(points[0]) if points[0] else 0}")
        if points[0]:
            sample_point = points[0][0]
            print(f"Sample point ID: {sample_point.id}")
            print(f"Sample point payload keys: {list(sample_point.payload.keys())}")
    else:
        print("No points stored in the collection yet")

except Exception as e:
    print(f"Error accessing Qdrant: {e}")