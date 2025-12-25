#!/usr/bin/env python3
"""
Railway startup script - handles PORT environment variable
"""
import os
import sys

if __name__ == "__main__":
    # Get PORT from environment, default to 8000
    port = int(os.environ.get("PORT", 8000))
    host = "0.0.0.0"

    print(f"Starting server on {host}:{port}")

    # Import and run uvicorn programmatically
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=host,
        port=port,
        log_level="info"
    )
