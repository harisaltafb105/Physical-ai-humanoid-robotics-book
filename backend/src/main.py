import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from .config import settings
from .database import engine, Base, DB_AVAILABLE
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

# CORS Configuration - Railway Production
# Allow all origins for Vercel frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
    max_age=3600,
)

# Include routers
app.include_router(chat_router)


@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"Environment: {settings.app_env}")
    logger.info(f"CORS Origins: {cors_origins}")

    # Validate configuration and log warnings
    config_warnings = settings.validate_production_config()
    if config_warnings:
        logger.warning("⚠️  Configuration warnings:")
        for warning in config_warnings:
            logger.warning(f"  - {warning}")

    # Initialize database tables if DB is available
    if DB_AVAILABLE and engine:
        try:
            async with engine.begin() as conn:
                await conn.run_sync(Base.metadata.create_all)
            logger.info("Database tables initialized")
        except Exception as e:
            logger.warning(f"Could not initialize database tables: {e}")

    # Check MCP server status (non-critical)
    try:
        status = mcp_server.get_service_status()
        for service, msg in status.items():
            logger.info(f"{service}: {msg}")
    except Exception as e:
        logger.warning(f"Could not get MCP service status: {e}")

    logger.info("API startup complete")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("Shutting down RAG Chatbot API...")


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    try:
        health = mcp_server.health_check()
        return {
            "status": "healthy",
            "services": health
        }
    except Exception as e:
        logger.error(f"Health check error: {e}")
        return {
            "status": "healthy",
            "services": {"error": str(e)}
        }


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "cors_origins": settings.cors_origins,
        "cors_origins_list": settings.cors_origins_list
    }
