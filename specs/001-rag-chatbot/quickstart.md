# Quickstart Guide: RAG Chatbot Local Development

**Feature**: 001-rag-chatbot
**Date**: 2025-12-10
**Purpose**: Get the RAG chatbot running locally for development and testing

## Prerequisites

- **Python**: 3.11 or higher
- **Node.js**: 18.x or higher (for Docusaurus frontend)
- **Git**: For cloning and version control
- **API Keys** (obtain before starting):
  - OpenAI API key (for GPT-4 and embeddings)
  - Qdrant Cloud API key and URL (free tier signup)
  - Neon Serverless Postgres connection string (free tier signup)

## Project Structure Overview

```
Physical-ai-human-robotics-book/
├── backend/                      # FastAPI backend (NEW)
│   ├── src/
│   │   ├── models/              # Data models (User, Conversation, Message)
│   │   ├── services/            # Business logic (RAG, personalization, translation)
│   │   ├── api/                 # API route handlers
│   │   ├── mcp/                 # MCP Server integration layer
│   │   └── main.py              # FastAPI app entry point
│   ├── tests/                   # Backend tests
│   ├── requirements.txt         # Python dependencies
│   └── .env.example             # Environment variables template
│
├── src/components/              # Docusaurus React components (NEW)
│   └── ChatWithBook.tsx         # Chatbot UI component
│
├── src/pages/                   # Docusaurus custom pages (NEW)
│   └── ai-chatbot.mdx           # Dedicated chatbot page
│
├── src/theme/                   # Docusaurus theme customization (NEW)
│   └── Root.tsx                 # Inject floating widget globally
│
├── scripts/                     # Utility scripts (NEW)
│   └── embed_book_content.py    # Embedding pipeline
│
├── docs/                        # Book content (existing)
│   ├── module-1-ros2/
│   ├── module-2-gazebo/
│   ├── module-3-isaac/
│   └── module-4-vla/
│
└── docusaurus.config.js         # Docusaurus config (existing)
```

## Step 1: Environment Setup

### 1.1 Create Backend Directory

```bash
mkdir -p backend/src/{models,services,api,mcp}
mkdir -p backend/tests
cd backend
```

### 1.2 Set Up Python Environment

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 1.3 Create `requirements.txt`

```txt
fastapi==0.109.0
uvicorn[standard]==0.25.0
pydantic==2.5.0
pydantic-settings==2.1.0
python-dotenv==1.0.0

# Authentication
better-auth==0.2.0  # Check actual package name
bcrypt==4.1.1
python-jose[cryptography]==3.3.0

# Database
psycopg2-binary==2.9.9
sqlalchemy==2.0.23

# Vector database
qdrant-client==1.7.0

# AI/LLM
openai==1.6.1

# Utilities
httpx==0.25.2
python-multipart==0.0.6
```

### 1.4 Configure Environment Variables

Create `backend/.env` (copy from `.env.example`):

```env
# Application
APP_ENV=development
DEBUG=True
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=http://localhost:3000

# OpenAI
OPENAI_API_KEY=sk-your-openai-api-key-here
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=book_content

# Neon Serverless Postgres
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/dbname?sslmode=require

# Better Auth (optional, generated automatically if not set)
AUTH_SECRET=your-32-char-random-secret-here

# Session management
SESSION_COOKIE_NAME=session_token
SESSION_MAX_AGE=604800  # 7 days in seconds

# Rate limiting
RATE_LIMIT_REQUESTS_PER_MINUTE=10
```

## Step 2: Initialize Database Schema

### 2.1 Run Database Migrations

```bash
cd backend

# Create database tables
python scripts/init_db.py
```

**Script: `backend/scripts/init_db.py`**

```python
import os
from sqlalchemy import create_engine, text
from dotenv import load_dotenv

load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")
engine = create_engine(DATABASE_URL)

schema_sql = """
-- (Copy from data-model.md SQL schema)
CREATE TABLE IF NOT EXISTS users (...);
CREATE TABLE IF NOT EXISTS conversations (...);
CREATE TABLE IF NOT EXISTS messages (...);
-- ... etc
"""

with engine.connect() as conn:
    conn.execute(text(schema_sql))
    conn.commit()
    print("✅ Database schema initialized")
```

### 2.2 Verify Database Connection

```bash
python -c "from sqlalchemy import create_engine; import os; engine = create_engine(os.getenv('DATABASE_URL')); print('✅ Database connected')"
```

## Step 3: Initialize Qdrant Collection

### 3.1 Create Qdrant Collection

```bash
cd backend

# Run collection initialization script
python scripts/init_qdrant.py
```

**Script: `backend/scripts/init_qdrant.py`**

```python
import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

# Create collection
client.create_collection(
    collection_name=collection_name,
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
)

print(f"✅ Qdrant collection '{collection_name}' created")
```

## Step 4: Run Embedding Pipeline

### 4.1 Embed Book Content

```bash
cd backend

# Run embedding pipeline (processes all docs/*.md files)
python scripts/embed_book_content.py
```

**Expected Output:**

```
Processing: docs/module-1-ros2/01-ros2-basics-first-node.md
  Created 8 chunks
  Embedded and uploaded to Qdrant
Processing: docs/module-1-ros2/02-topics-services.md
  Created 6 chunks
  Embedded and uploaded to Qdrant
...
✅ Embedding pipeline complete: 250 chunks processed
```

### 4.2 Verify Embeddings

```bash
python scripts/check_qdrant.py
```

**Output:**

```
Qdrant collection 'book_content' info:
  Total vectors: 250
  Total points: 250
  Status: green
✅ Embeddings verified
```

## Step 5: Start FastAPI Backend

### 5.1 Run Development Server

```bash
cd backend

# Start FastAPI with auto-reload
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output:**

```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345]
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 5.2 Test API Health

```bash
curl http://localhost:8000/health

# Expected response:
{"status": "healthy", "services": {"qdrant": "connected", "database": "connected", "openai": "connected"}}
```

### 5.3 Explore API Documentation

Open in browser:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## Step 6: Set Up Docusaurus Frontend

### 6.1 Install Dependencies

```bash
# From project root
npm install
```

### 6.2 Create ChatWithBook Component

**File: `src/components/ChatWithBook.tsx`**

```tsx
import React, { useState, useEffect } from 'react';
import './ChatWithBook.css';

export default function ChatWithBook() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isOpen, setIsOpen] = useState(false);

  const sendMessage = async () => {
    // TODO: Implement API call to backend
    const response = await fetch('http://localhost:8000/api/chat/conversations/{id}/messages', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ content: input })
    });
    const data = await response.json();
    setMessages([...messages, data.user_message, data.assistant_message]);
    setInput('');
  };

  return (
    <div className={`chat-widget ${isOpen ? 'open' : 'minimized'}`}>
      {/* Chat UI implementation */}
      <button onClick={() => setIsOpen(!isOpen)}>
        {isOpen ? 'Minimize' : 'Chat'}
      </button>
      {/* ... rest of chat UI */}
    </div>
  );
}
```

### 6.3 Inject Floating Widget Globally

**File: `src/theme/Root.tsx` (swizzle Docusaurus root)**

```bash
npm run swizzle @docusaurus/theme-classic Root -- --wrap
```

**Edit `src/theme/Root.tsx`:**

```tsx
import React from 'react';
import ChatWithBook from '@site/src/components/ChatWithBook';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWithBook />
    </>
  );
}
```

### 6.4 Create Dedicated Chatbot Page

**File: `src/pages/ai-chatbot.mdx`**

```mdx
---
title: AI Chatbot
description: Chat with the Physical AI & Humanoid Robotics Book
---

# AI Chatbot

Ask questions about ROS2, Gazebo, Isaac Sim, and VLA integration!

import ChatWithBook from '@site/src/components/ChatWithBook';

<ChatWithBook fullScreen={true} />
```

### 6.5 Update Docusaurus Config

**Edit `docusaurus.config.js`:**

```js
module.exports = {
  // ... existing config
  themeConfig: {
    navbar: {
      items: [
        // ... existing items
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Docs',
        },
        {
          to: '/ai-chatbot',
          label: 'AI Chatbot',
          position: 'right',
        },
      ],
    },
  },
};
```

## Step 7: Run Full Stack Locally

### 7.1 Start Backend

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

### 7.2 Start Docusaurus (in separate terminal)

```bash
npm run start
```

**Expected Output:**

```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### 7.3 Access Application

- **Docusaurus Site**: http://localhost:3000
- **Chatbot Page**: http://localhost:3000/ai-chatbot
- **Backend API Docs**: http://localhost:8000/docs

## Step 8: Test Core Features

### 8.1 Test Anonymous Chatbot

1. Open http://localhost:3000
2. Click the floating chat widget
3. Type: "What is ROS2?"
4. Verify response includes book citations

### 8.2 Test Signup/Signin

1. Navigate to http://localhost:3000/ai-chatbot
2. Click "Sign Up"
3. Fill form:
   - Email: test@example.com
   - Password: TestPass123!
   - Software Background: Beginner
   - Hardware Background: Hobbyist
4. Verify account created and signed in

### 8.3 Test Personalization

1. Sign in as beginner/hobbyist user
2. Navigate to any chapter (e.g., Module 1: ROS2 Basics)
3. Click "Personalize" button at top of chapter
4. Verify content regenerates with simplified explanations

### 8.4 Test Urdu Translation

1. Sign in
2. Navigate to any chapter
3. Click "Translate to Urdu" button
4. Verify:
   - Text is in Urdu
   - Code blocks remain in English
   - Technical terms preserved

## Troubleshooting

### Issue: "Database connection failed"

**Solution:**
```bash
# Verify DATABASE_URL in .env
# Check Neon Postgres dashboard is active
# Test connection:
psql $DATABASE_URL -c "SELECT 1"
```

### Issue: "Qdrant API error: Unauthorized"

**Solution:**
```bash
# Verify QDRANT_API_KEY in .env
# Check Qdrant Cloud dashboard for correct API key
```

### Issue: "OpenAI rate limit exceeded"

**Solution:**
```bash
# Add exponential backoff in backend/src/services/openai_service.py
# Reduce concurrent requests in development
```

### Issue: "Embedding pipeline takes too long"

**Solution:**
```bash
# Run pipeline for single chapter first:
python scripts/embed_book_content.py --chapter docs/module-1-ros2/01-ros2-basics-first-node.md

# Then run full pipeline:
python scripts/embed_book_content.py --all
```

## Development Workflow

### Adding New Features

1. Update spec.md with new requirements
2. Run `/sp.plan` to update architecture
3. Run `/sp.tasks` to generate implementation tasks
4. Implement tasks in order
5. Run tests: `pytest backend/tests`
6. Test manually in browser
7. Commit changes

### Testing

```bash
# Backend tests
cd backend
pytest tests/ -v

# Frontend tests (if added)
npm run test

# Integration tests
pytest tests/integration/ -v
```

### Code Quality

```bash
# Python linting
cd backend
black src/
flake8 src/

# TypeScript/React linting
npm run lint
npm run format
```

## Next Steps

1. Implement authentication endpoints (FR-019, FR-020)
2. Implement chatbot query endpoint (FR-003, FR-004)
3. Implement personalization endpoint (FR-022)
4. Implement translation endpoint (FR-023)
5. Build ChatWithBook React component UI
6. Add error boundaries and loading states
7. Write unit and integration tests
8. Deploy to production (see deployment guide)

## Useful Commands

```bash
# Backend
uvicorn src.main:app --reload              # Start dev server
pytest tests/ -v --cov                     # Run tests with coverage
python scripts/embed_book_content.py       # Re-run embedding pipeline

# Frontend
npm run start                              # Start Docusaurus dev server
npm run build                              # Build for production
npm run serve                              # Serve production build

# Database
psql $DATABASE_URL                         # Connect to Neon Postgres
python scripts/reset_db.py                 # Reset database (dev only)

# Qdrant
python scripts/check_qdrant.py             # Verify collection status
python scripts/clear_qdrant.py             # Clear all embeddings (dev only)
```

## Resources

- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Docusaurus Docs**: https://docusaurus.io/
- **OpenAI API Docs**: https://platform.openai.com/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Neon Postgres Docs**: https://neon.tech/docs
- **Better Auth Docs**: https://better-auth.com/ (check actual URL)

---

**Last Updated**: 2025-12-10
**Maintained By**: Development Team
