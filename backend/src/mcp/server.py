import logging
from typing import Dict, Any
from .openai_proxy import openai_proxy
from .qdrant_proxy import qdrant_proxy
from .neon_proxy import neon_proxy

logger = logging.getLogger(__name__)


class MCPServer:
    """
    Model Context Protocol Server - Unified service proxy.
    Centralizes access to external services (OpenAI, Qdrant, Neon).
    """
    
    def __init__(self):
        self.openai = openai_proxy
        self.qdrant = qdrant_proxy
        self.neon = neon_proxy
        logger.info("MCP Server initialized")
    
    def health_check(self) -> Dict[str, Any]:
        """
        Check health of all external services.
        
        Returns:
            Dict with service statuses
        """
        return {
            "openai": "connected" if self.openai.check_connection() else "disconnected",
            "qdrant": "connected" if self.qdrant.check_connection() else "disconnected",
            "database": "connected" if self.neon.check_connection() else "fallback"
        }
    
    def get_service_status(self) -> Dict[str, str]:
        """
        Get detailed status of each service.
        
        Returns:
            Dict mapping service names to status messages
        """
        status = {}
        
        # OpenAI
        try:
            if self.openai.check_connection():
                status["openai"] = "✅ Connected"
            else:
                status["openai"] = "❌ Disconnected"
        except Exception as e:
            status["openai"] = f"❌ Error: {str(e)}"
        
        # Qdrant
        try:
            info = self.qdrant.get_collection_info()
            status["qdrant"] = f"✅ Connected ({info['vectors_count']} vectors)"
        except Exception as e:
            status["qdrant"] = f"❌ Error: {str(e)}"
        
        # Neon
        if self.neon.use_fallback:
            status["database"] = "⚠️  Using in-memory fallback"
        elif self.neon.check_connection():
            status["database"] = "✅ Connected"
        else:
            status["database"] = "❌ Disconnected"
        
        return status


# Global instance
mcp_server = MCPServer()
