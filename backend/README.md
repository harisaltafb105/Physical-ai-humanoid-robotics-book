# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics Book chatbot with authentication, personalization, and Urdu translation.

## Quick Start

### 1. Setup Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Copy `.env.example` to `.env` and fill in your API keys:

```bash
cp .env.example .env
# Edit .env with your credentials
```

### 3. Initialize Database

```bash
python scripts/init_db.py
```

### 4. Initialize Qdrant Collection

```bash
python scripts/init_qdrant.py
```

### 5. Run Embedding Pipeline

```bash
python scripts/embed_book_content.py
```

### 6. Start Development Server

```bash
uvicorn src.main:app --reload --port 8000
```

API Documentation: http://localhost:8000/docs

## Project Structure

```
backend/
├── src/
│   ├── models/          # SQLAlchemy & Pydantic models
│   ├── services/        # Business logic
│   ├── api/             # API route handlers
│   ├── mcp/             # MCP Server integration
│   ├── config.py        # Configuration management
│   ├── database.py      # Database setup
│   └── main.py          # FastAPI app
├── tests/               # Unit, integration, contract tests
├── scripts/             # Utility scripts
└── migrations/          # Database migrations
```

## API Endpoints

- **Auth**: `/api/auth/*` (signup, signin, signout, me)
- **Chat**: `/api/chat/*` (conversations, messages)
- **Content**: `/api/content/*` (personalize, translate, original)
- **Health**: `/health` (service health check)

## Testing

```bash
pytest tests/ -v
pytest tests/ -v --cov  # With coverage
```

## Deployment

### Docker

```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

## Environment Variables

See `.env.example` for required configuration.

## License

MIT
