import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

if not QDRANT_URL or not QDRANT_API_KEY:
    print("[ERROR] QDRANT_URL or QDRANT_API_KEY not found in environment variables")
    sys.exit(1)

try:
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )
    
    # Check if collection already exists
    collections = client.get_collections().collections
    existing_collection = any(c.name == COLLECTION_NAME for c in collections)
    
    if existing_collection:
        print(f"[WARNING] Collection '{COLLECTION_NAME}' already exists. Skipping creation.")
    else:
        # Create collection with 1536-dimensional vectors (text-embedding-3-small)
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
        )
        print(f"[SUCCESS] Qdrant collection '{COLLECTION_NAME}' created successfully")
        print(f"   Vector size: 1536 dimensions")
        print(f"   Distance metric: Cosine")

    # Verify collection
    collection_info = client.get_collection(COLLECTION_NAME)
    print(f"[SUCCESS] Collection verified: {collection_info.vectors_count} vectors")

except Exception as e:
    print(f"[ERROR] Error initializing Qdrant collection: {e}")
    sys.exit(1)
