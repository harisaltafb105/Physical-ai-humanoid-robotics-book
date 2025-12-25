#!/bin/bash
# Railway startup script - binds to Railway's dynamic PORT

# Use Railway's PORT env var, fallback to 8000 for local dev
PORT=${PORT:-8000}

echo "Starting FastAPI server on 0.0.0.0:$PORT"
uvicorn src.main:app --host 0.0.0.0 --port $PORT
