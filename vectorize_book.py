import os
import requests
import json
from pathlib import Path
import uuid

def vectorize_book():
    # Base directory for the book content
    docs_dir = "frontend_book/my-book/docs"

    # Vectorize endpoint
    vectorize_url = "http://localhost:8000/api/chat/vectorize"

    # Check if docs directory exists
    if not os.path.exists(docs_dir):
        print(f"[ERROR] Directory {docs_dir} does not exist")
        return

    # Find all markdown files recursively
    markdown_files = list(Path(docs_dir).rglob("*.md"))

    if not markdown_files:
        print(f"[ERROR] No markdown files found in {docs_dir}")
        return

    print(f"[INFO] Found {len(markdown_files)} markdown files to process")

    success_count = 0
    failure_count = 0

    for file_path in markdown_files:
        try:
            # Read the file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Generate a UUID for the content_id (required by Qdrant)
            content_id = str(uuid.uuid4())

            # Prepare the payload
            payload = {
                "content_id": content_id,
                "content": content,
                "metadata": {
                    "chapter": os.path.basename(os.path.dirname(file_path)),
                    "section": os.path.splitext(os.path.basename(file_path))[0],
                    "tags": [],
                    "file_path": str(file_path)  # Store the original file path in metadata
                }
            }

            # Send POST request to vectorize endpoint
            response = requests.post(
                vectorize_url,
                json=payload,
                headers={"Content-Type": "application/json"}
            )

            if response.status_code == 200:
                print(f"[SUCCESS] Vectorized: {file_path}")
                success_count += 1
            else:
                print(f"[FAILED] Failed: {file_path}")
                print(f"   Response: {response.status_code} - {response.text}")
                failure_count += 1

        except Exception as e:
            print(f"[FAILED] Failed: {file_path}")
            print(f"   Error: {str(e)}")
            failure_count += 1

    print(f"\n[SUMMARY] Summary: {success_count} successful, {failure_count} failed")
    return success_count, failure_count

if __name__ == "__main__":
    vectorize_book()