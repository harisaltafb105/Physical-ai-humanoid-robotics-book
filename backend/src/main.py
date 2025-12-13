import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from .config import settings
from .database import engine, Base
from .mcp.server import mcp_server
from .api.chat import router as chat_router

logging.basicConfig(
    level=logging.DEBUG if settings.debug else logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

limiter = Limiter(key_func=get_remote_address)

app = FastAPI(
    title="RAG Chatbot API",
    description="Physical AI & Humanoid Robotics Book Chatbot",
    version="1.0.0",
    debug=settings.debug
)

app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router)


@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting RAG Chatbot API...")
    
    status = mcp_server.get_service_status()
    for service, msg in status.items():
        logger.info(f"{service}: {msg}")
    
    logger.info("API startup complete")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("Shutting down RAG Chatbot API...")


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    health = mcp_server.health_check()
    return {
        "status": "healthy",
        "services": health
    }


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs"
    }
