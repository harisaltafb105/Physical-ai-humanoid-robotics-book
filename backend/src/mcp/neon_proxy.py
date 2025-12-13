import logging
from typing import Any, Dict, List, Optional, Tuple
from sqlalchemy import create_engine, text
from sqlalchemy.exc import OperationalError
from ..config import settings

logger = logging.getLogger(__name__)


class NeonProxy:
    """
    Proxy for Neon Postgres database with fallback to in-memory storage.
    Provides graceful degradation when database is unavailable.
    """
    
    def __init__(self):
        self.database_url = settings.database_url
        self.engine = None
        self.in_memory_storage = {}  # Fallback storage
        self.use_fallback = False
        
        try:
            self.engine = create_engine(
                self.database_url,
                pool_pre_ping=True,
                echo=settings.debug
            )
            # Test connection
            if self.check_connection():
                logger.info("✅ Neon Postgres connected successfully")
            else:
                self._activate_fallback()
        except Exception as e:
            logger.error(f"Failed to create Neon engine: {e}")
            self._activate_fallback()
    
    def _activate_fallback(self):
        """Activate in-memory fallback storage."""
        self.use_fallback = True
        logger.warning("⚠️  Using in-memory storage fallback (data will not persist)")
    
    def execute_query(
        self,
        sql: str,
        params: Optional[Dict[str, Any]] = None,
        fetch: bool = True
    ) -> Tuple[bool, Optional[List[Dict[str, Any]]]]:
        """
        Execute SQL query with optional parameters.
        
        Args:
            sql: SQL query string
            params: Optional dict of query parameters
            fetch: Whether to fetch results (SELECT queries)
            
        Returns:
            Tuple of (success: bool, results: Optional[List[Dict]])
        """
        if self.use_fallback:
            logger.warning("Query executed against in-memory fallback (not persisted)")
            return True, []
        
        try:
            with self.engine.connect() as conn:
                result = conn.execute(text(sql), params or {})
                conn.commit()
                
                if fetch:
                    rows = result.fetchall()
                    # Convert to list of dicts
                    data = [dict(row._mapping) for row in rows]
                    return True, data
                else:
                    return True, None
                    
        except Exception as e:
            logger.error(f"Neon query execution error: {e}")
            return False, None
    
    def check_connection(self) -> bool:
        """
        Verify database connection.
        
        Returns:
            True if connection successful, False otherwise
        """
        if self.use_fallback:
            return False
            
        try:
            with self.engine.connect() as conn:
                conn.execute(text("SELECT 1"))
                return True
        except OperationalError as e:
            logger.error(f"Neon connection check failed: {e}")
            return False
    
    def get_or_create_session(self, session_id: str) -> Dict[str, Any]:
        """
        Get or create a session in fallback storage.
        
        Args:
            session_id: Unique session identifier
            
        Returns:
            Session data dict
        """
        if session_id not in self.in_memory_storage:
            self.in_memory_storage[session_id] = {
                "id": session_id,
                "conversations": [],
                "user": None
            }
        return self.in_memory_storage[session_id]
    
    def store_in_memory(self, key: str, value: Any):
        """
        Store data in fallback memory.
        
        Args:
            key: Storage key
            value: Data to store
        """
        self.in_memory_storage[key] = value
        logger.debug(f"Stored in memory: {key}")
    
    def retrieve_from_memory(self, key: str) -> Optional[Any]:
        """
        Retrieve data from fallback memory.
        
        Args:
            key: Storage key
            
        Returns:
            Stored value or None
        """
        return self.in_memory_storage.get(key)


# Global instance
neon_proxy = NeonProxy()
