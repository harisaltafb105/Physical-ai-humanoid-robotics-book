# Research: RAG Chatbot with Auth, Personalization & Translation

**Feature**: 001-rag-chatbot
**Date**: 2025-12-10
**Purpose**: Resolve technical decisions for RAG chatbot implementation with Better Auth, personalization, and Urdu translation

## Research Areas

### 1. RAG Architecture with OpenAI Agents/ChatKit SDK

**Decision**: Use OpenAI Assistants API with Function Calling + Qdrant vector store

**Rationale**:
- OpenAI Assistants API provides built-in conversation management and context handling
- Function calling enables structured tool use for vector search, personalization, and translation
- Better integration with streaming responses for real-time user experience
- ChatKit SDK would require more custom conversation state management

**Alternatives Considered**:
- **LangChain + OpenAI**: More flexible but adds complexity and dependency overhead
- **ChatKit SDK**: Proprietary, less documentation, harder to customize for Urdu translation
- **Custom RAG Pipeline**: Too much implementation overhead for core functionality

**Implementation Approach**:
- FastAPI endpoints expose chatbot API
- OpenAI Assistants API manages conversation threads
- Custom functions for: `search_book_content`, `personalize_chapter`, `translate_to_urdu`
- Vector search via Qdrant client library (Python SDK)

---

### 2. Embedding Model for Qdrant

**Decision**: Use `text-embedding-3-small` (OpenAI) for cost efficiency on free tier

**Rationale**:
- Qdrant Cloud Free Tier: 1GB storage, sufficient for ~50 book chapters
- `text-embedding-3-small`: 1536 dimensions, optimized for cost/performance balance
- Consistent with OpenAI Assistants API ecosystem
- Supports semantic search for both English and Urdu queries (multilingual capable)

**Alternatives Considered**:
- **text-embedding-3-large**: Higher accuracy but 3x cost, unnecessary for book content
- **Sentence-Transformers (open-source)**: Requires separate hosting, adds infrastructure complexity
- **BGE-M3 (multilingual)**: Better for Urdu but no free tier with comparable vector DB

**Chunking Strategy**:
- Chunk size: 800 tokens with 200 token overlap
- Preserve Markdown structure: headings, code blocks, lists as metadata
- Metadata fields: `chapter_id`, `module_name`, `heading_hierarchy`, `has_code`

---

### 3. Better Auth Integration (No API Keys Required)

**Decision**: Use Better Auth with email/password provider and Neon Postgres adapter

**Rationale**:
- Better Auth is designed for Next.js but can work with FastAPI via REST API bridge
- Email/password provider requires no external API keys (GitHub, Google providers need keys)
- Neon Postgres stores user profiles and sessions natively
- Supports custom fields for software/hardware background during signup

**Alternatives Considered**:
- **NextAuth.js**: Requires Next.js frontend, not compatible with Docusaurus
- **Auth0**: Requires API keys and external service dependency
- **Custom JWT Auth**: Secure but requires building entire auth flow from scratch

**Implementation Approach**:
- Better Auth runs as middleware in FastAPI backend
- Custom registration endpoint collects `email`, `password`, `software_bg`, `hardware_bg`
- Session tokens stored in HTTP-only cookies for security
- Neon Postgres schema: `users(id, email, password_hash, software_bg, hardware_bg, created_at)`

---

### 4. Personalization Engine

**Decision**: Use OpenAI GPT-4 Turbo with system prompts tailored to user background

**Rationale**:
- Dynamic content generation based on `software_bg` and `hardware_bg` fields
- System prompt templates:
  - Beginner Software: "Explain concepts with analogies, define technical terms, provide step-by-step instructions"
  - Advanced Software: "Focus on best practices, optimization techniques, and edge cases"
  - Simulation-only Hardware: "Emphasize Gazebo and Isaac Sim examples, minimize physical hardware requirements"
  - Professional Hardware: "Include calibration, real-world deployment considerations, and troubleshooting"
- Regenerates chapter content on-demand (not pre-cached to save storage)

**Alternatives Considered**:
- **Pre-cached Personalization**: Generates 9 versions (3 software × 3 hardware) upfront - storage prohibitive
- **Rule-based Templates**: Less flexible, harder to maintain, lower quality output
- **Fine-tuned Model**: Requires training data and hosting infrastructure

**Caching Strategy**:
- Store personalized content in user session (Redis or in-memory for free tier)
- TTL: 1 hour per personalized chapter
- Invalidate on chapter content update

---

### 5. Urdu Translation

**Decision**: Use OpenAI GPT-4 Turbo with custom prompts for technical content preservation

**Rationale**:
- GPT-4 Turbo supports Urdu with high fluency for technical content
- Custom prompt ensures code blocks remain unchanged
- Can handle markdown formatting preservation
- Context-aware translation (understands robotics terminology)

**Alternatives Considered**:
- **Google Translate API**: Free tier limited (500k chars/month), poor technical term handling
- **DeepL API**: No Urdu support
- **MarianMT (open-source)**: Requires hosting, lower quality for technical content

**Translation Prompts**:
```
Translate the following robotics book chapter from English to Urdu.
CRITICAL RULES:
1. Keep all code blocks exactly as-is (do not translate code)
2. Keep technical terms in English when no standard Urdu equivalent exists (e.g., "URDF", "ROS2 node")
3. Translate explanatory text naturally for Urdu-speaking readers
4. Preserve all Markdown formatting (headings, lists, links)
```

**Caching Strategy**:
- Same as personalization: session-based with 1-hour TTL
- Separate cache key: `translated_{user_id}_{chapter_id}`

---

### 6. MCP Server Integration (Context Level 7)

**Decision**: Create unified MCP server as FastAPI middleware/proxy for external services

**Rationale**:
- MCP Server acts as single integration point for Qdrant, Neon, and OpenAI
- Centralizes API key management and rate limiting
- Provides consistent error handling and retry logic
- Supports graceful degradation when services unavailable

**Implementation Approach**:
- MCP Server exposes internal APIs:
  - `/mcp/qdrant/search` → Qdrant vector search
  - `/mcp/neon/user_profile` → Neon Postgres queries
  - `/mcp/openai/chat` → OpenAI API calls
- FastAPI backend calls MCP endpoints instead of direct service APIs
- Environment variables for service credentials managed at MCP Server level

**Alternatives Considered**:
- **Direct Service Calls**: No centralized control, harder to test and mock
- **Service Mesh (e.g., Istio)**: Over-engineered for this project scale
- **API Gateway (e.g., Kong)**: Adds deployment complexity

---

### 7. Docusaurus Integration for ChatWithBook Component

**Decision**: Create React component with Docusaurus theme swizzling for global integration

**Rationale**:
- Docusaurus supports custom React components via MDX and swizzling
- Theme swizzling allows injecting floating widget into root layout
- Component can access Docusaurus context (current page, navigation state)
- SSR-compatible with client-side hydration for chat functionality

**Implementation Approach**:
- `src/components/ChatWithBook.tsx`: Main chat UI component
- `src/theme/Root.tsx` (swizzled): Wraps app to inject floating widget
- Communication with FastAPI backend via REST API + WebSocket for streaming
- State management: React Context API for conversation history
- Dedicated page: `docs/ai-chatbot.mdx` imports `ChatWithBook` component

**Alternatives Considered**:
- **Separate Iframe**: Poor UX, security issues, harder state management
- **Browser Extension**: Users must install, reduces adoption
- **Docusaurus Plugin**: More complex to develop and maintain

---

### 8. Neon Serverless Postgres Schema Design

**Decision**: Minimal schema with graceful degradation to in-memory storage

**Rationale**:
- Neon Free Tier: 0.5GB storage, 3 active branches - sufficient for user profiles and conversations
- Schema designed for optional persistence (system works without SQL)
- Connection pooling via `psycopg2-pool` for serverless compatibility

**Schema**:
```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  software_bg VARCHAR(50) CHECK (software_bg IN ('beginner', 'intermediate', 'advanced')),
  hardware_bg VARCHAR(50) CHECK (hardware_bg IN ('simulation-only', 'hobbyist', 'professional')),
  created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE conversations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  title TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  conversation_id UUID REFERENCES conversations(id) ON DELETE CASCADE,
  role VARCHAR(20) CHECK (role IN ('user', 'assistant')),
  content TEXT NOT NULL,
  citations JSONB,
  created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_conversations_user ON conversations(user_id, updated_at DESC);
CREATE INDEX idx_messages_conversation ON messages(conversation_id, created_at);
```

**Alternatives Considered**:
- **MongoDB (Neon alternative)**: Free tier comparable, but SQL better for relational user/conversation data
- **SQLite**: No cloud hosting, requires file storage management
- **Supabase**: Requires API keys, less generous free tier

---

### 9. Chunking Pipeline for Markdown Files

**Decision**: Offline batch processing with incremental update detection

**Rationale**:
- Run pipeline script: `python scripts/embed_book_content.py`
- Detects new/modified markdown files by comparing file hashes
- Generates chunks, creates embeddings, uploads to Qdrant
- Metadata extraction: frontmatter (id, title), heading hierarchy, code block detection

**Implementation**:
```python
# Pseudo-code structure
def chunk_markdown(file_path):
    parse_frontmatter()  # Extract chapter metadata
    split_by_headings()  # Preserve semantic boundaries
    extract_code_blocks()  # Tag chunks containing code
    create_overlapping_chunks(size=800, overlap=200)
    return chunks_with_metadata

def embed_and_upload(chunks):
    embeddings = openai.embeddings.create(model="text-embedding-3-small", input=chunks)
    qdrant.upsert(collection="book_content", vectors=embeddings, payloads=metadata)
```

**Trigger**:
- Manual: `npm run embed-content`
- Automatic: GitHub Action on content file changes (future enhancement)

**Alternatives Considered**:
- **Real-time Embedding**: Too slow for content updates, wastes API calls
- **LlamaIndex**: Adds dependency overhead for simple use case
- **Semantic Chunking (LangChain)**: Over-engineered, fixed-size works well for book content

---

### 10. Error Handling and Graceful Degradation

**Decision**: Layered fallback strategy with user-facing error messages

**Rationale**:
- Better Auth unavailable → Allow anonymous chatbot access (no personalization/translation)
- Neon Postgres unavailable → Use in-memory session storage (lose history on restart)
- Qdrant unavailable → Return error message to user: "Search temporarily unavailable"
- OpenAI rate limit → Implement exponential backoff (3 retries, max 10s wait)

**Error Boundaries**:
- Frontend: React Error Boundary catches component crashes
- Backend: FastAPI exception handlers return structured JSON errors
- MCP Server: Circuit breaker pattern for external service failures

**Alternatives Considered**:
- **Fail Fast**: Poor UX, feature becomes unusable
- **Silent Failure**: Confusing for users, hard to debug
- **Retry Forever**: Wastes resources, increases latency

---

## Technology Summary

| Component | Technology | Version | Justification |
|-----------|-----------|---------|---------------|
| Backend | FastAPI | 0.109+ | Async support, OpenAPI docs, lightweight |
| Auth | Better Auth | Latest | No API keys required, Neon Postgres integration |
| LLM | OpenAI GPT-4 Turbo | gpt-4-turbo-preview | Function calling, Urdu support, streaming |
| Embeddings | OpenAI | text-embedding-3-small | Cost-effective, 1536d, multilingual |
| Vector DB | Qdrant Cloud | Free Tier | 1GB storage, Python SDK, filters support |
| Database | Neon Serverless Postgres | Free Tier | 0.5GB storage, auto-scaling, Postgres compatibility |
| Frontend | React + Docusaurus | 3.0+ | SSR support, MDX, theme customization |
| Translation | OpenAI GPT-4 Turbo | gpt-4-turbo-preview | High-quality Urdu, context-aware |
| Personalization | OpenAI GPT-4 Turbo | gpt-4-turbo-preview | Dynamic prompts, background-aware generation |
| MCP Server | FastAPI Middleware | N/A | Unified service proxy |

---

## Performance Estimates

Based on free tier limits and average usage:

| Operation | Expected Time | Free Tier Limit | Notes |
|-----------|---------------|-----------------|-------|
| Signup/Signin | < 3s | N/A | Neon Postgres query + password hash |
| Chatbot Query | < 5s | 500 req/min (OpenAI) | Vector search (50ms) + LLM (3-4s) |
| Personalization | < 10s | 500 req/min | Regenerate 2000-word chapter |
| Urdu Translation | < 8s | 500 req/min | Translate 2000-word chapter |
| Embedding Pipeline | < 10min | 3000 req/min | 50 chapters × 5 chunks/chapter = 250 embeddings |
| Concurrent Users | 50+ | Qdrant: 100 RPS | Assuming 1 query/user/5s |

---

## Security Considerations

1. **API Keys**: All credentials in environment variables, never committed to Git
2. **Authentication**: HTTP-only cookies prevent XSS attacks on tokens
3. **Input Validation**: Sanitize all user inputs (queries, selected text) before LLM calls
4. **Rate Limiting**: Per-user limits (10 queries/minute) to prevent abuse
5. **CORS**: Restrict FastAPI CORS to Docusaurus site domain only
6. **SQL Injection**: Use parameterized queries via SQLAlchemy ORM
7. **Password Storage**: bcrypt hashing with salt (Better Auth handles this)

---

## Open Questions Resolved

1. **Q**: How to handle multi-turn conversations with context?
   **A**: OpenAI Assistants API maintains thread state automatically

2. **Q**: Can Urdu translation preserve code syntax highlighting?
   **A**: Yes, by instructing LLM to skip code blocks; Docusaurus handles highlighting

3. **Q**: How to detect when book content is updated?
   **A**: File hash comparison in embedding pipeline script

4. **Q**: What if user selects both beginner software + professional hardware?
   **A**: Personalization system prompt combines both: "beginner-level explanations with professional hardware deployment guidance"

5. **Q**: How to test Urdu translation quality without native speakers?
   **A**: Use back-translation (Urdu → English) and compare to original; manual review by community

---

## Next Steps (Phase 1)

1. Design data model schema (data-model.md)
2. Define API contracts for FastAPI endpoints (contracts/)
3. Create quickstart guide for local development (quickstart.md)
4. Update agent context with selected technologies
