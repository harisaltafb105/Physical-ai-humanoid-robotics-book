import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, ScoredPoint, Filter
from ..config import settings

logger = logging.getLogger(__name__)


class QdrantProxy:
    """
    Proxy for Qdrant vector database operations.
    Handles vector search, insertion, and collection management.
    """
    
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection_name
    
    def search_vectors(
        self,
        query_vector: List[float],
        limit: int = 5,
        filter_conditions: Optional[Filter] = None
    ) -> List[ScoredPoint]:
        """
        Search for similar vectors in the collection.

        Args:
            query_vector: Query embedding vector (1536 dimensions)
            limit: Maximum number of results to return
            filter_conditions: Optional Qdrant filter for metadata

        Returns:
            List of ScoredPoint objects with payload and score
        """
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit,
                query_filter=filter_conditions
            )
            # Extract points from QueryResponse
            points = results.points if hasattr(results, 'points') else results
            logger.info(f"Qdrant search returned {len(points)} results")
            return points

        except Exception as e:
            logger.error(f"Qdrant search error: {e}")
            raise
    
    def upsert_vectors(self, points: List[PointStruct]) -> bool:
        """
        Insert or update vectors in the collection.
        
        Args:
            points: List of PointStruct objects with id, vector, and payload
            
        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Upserted {len(points)} vectors to Qdrant")
            return True
            
        except Exception as e:
            logger.error(f"Qdrant upsert error: {e}")
            return False
    
    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection information and statistics.

        Returns:
            Dict with collection metadata
        """
        try:
            collection = self.client.get_collection(self.collection_name)
            # Handle different Qdrant client versions
            points_count = getattr(collection, 'points_count', 0)
            if hasattr(collection, 'vectors_count'):
                vectors_count = collection.vectors_count
            else:
                vectors_count = points_count  # Fallback

            # Extract vector size from config
            vector_size = None
            if hasattr(collection, 'config') and hasattr(collection.config, 'params'):
                params = collection.config.params
                if hasattr(params, 'vectors'):
                    if hasattr(params.vectors, 'size'):
                        vector_size = params.vectors.size
                    elif isinstance(params.vectors, dict):
                        # Multi-vector config
                        vector_size = next(iter(params.vectors.values())).size if params.vectors else None

            return {
                "name": self.collection_name,
                "vectors_count": vectors_count,
                "points_count": points_count,
                "status": str(collection.status) if hasattr(collection, 'status') else "unknown",
                "vector_size": vector_size
            }

        except Exception as e:
            logger.error(f"Qdrant get collection info error: {e}")
            raise
    
    def delete_vectors(self, ids: List[str]) -> bool:
        """
        Delete vectors by IDs.
        
        Args:
            ids: List of point IDs to delete
            
        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=ids
            )
            logger.info(f"Deleted {len(ids)} vectors from Qdrant")
            return True
            
        except Exception as e:
            logger.error(f"Qdrant delete error: {e}")
            return False
    
    def check_connection(self) -> bool:
        """
        Verify Qdrant connection.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.get_collection_info()
            return True
        except Exception as e:
            logger.error(f"Qdrant connection check failed: {e}")
            return False


# Global instance
qdrant_proxy = QdrantProxy()
