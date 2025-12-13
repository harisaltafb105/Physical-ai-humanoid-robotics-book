# Implementation Plan: RAG Chatbot with Auth, Personalization & Translation

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

## Summary

Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the Physical AI & Humanoid Robotics Docusaurus book. The system integrates:

- **Core RAG**: OpenAI Assistants API for chat, Qdrant Cloud for vector search, text-embedding-3-small for embeddings
- **Authentication**: Better Auth with email/password, collecting software & hardware background profiles
- **Personalization**: GPT-4 Turbo regenerates chapter content based on user background (beginner/intermediate/advanced × simulation-only/hobbyist/professional)
- **Translation**: GPT-4 Turbo translates chapters to Urdu while preserving code blocks and technical terms
- **Infrastructure**: FastAPI backend, Neon Serverless Postgres for user/conversation data, MCP Server for unified external service integration, Docusaurus React components for UI

**Technical Approach**: Web application with FastAPI backend exposing REST API, React-based Docusaurus frontend with custom components (ChatWithBook) and theme swizzling for global floating widget. All external services (Qdrant, Neon, OpenAI) accessed through MCP Server proxy layer. System designed for graceful degradation: chatbot functions without auth/persistence when Neon/Better Auth unavailable.

## Technical Context

**Language/Version**: Python 3.11+, TypeScript 5.x (React/Docusaurus), Node.js 18+
**Primary Dependencies**:
- Backend: FastAPI 0.109+, OpenAI SDK 1.6+, Qdrant Client 1.7+, Better Auth 0.2+, SQLAlchemy 2.0+, psycopg2 2.9+
- Frontend: Docusaurus 3.0+, React 18+, TypeScript 5+
- AI/ML: OpenAI GPT-4 Turbo (gpt-4-turbo-preview), text-embedding-3-small (1536 dimensions)

**Storage**:
- Primary: Neon Serverless Postgres (user profiles, conversations, messages)
- Vector: Qdrant Cloud (embedded book chunks, 1GB free tier)
- Cache: In-memory LRU cache for personalized/translated content (1-hour TTL)
- Fallback: Session storage when Postgres unavailable

**Testing**: pytest (backend unit/integration), Jest + React Testing Library (frontend), API contract testing via OpenAPI spec
**Target Platform**: Linux server (backend), modern browsers (Chrome, Firefox, Safari, Edge) for frontend
**Project Type**: Web application (FastAPI backend + Docusaurus frontend)
**Performance Goals**:
- Chatbot response: < 5 seconds (p95)
- Signup/signin: < 3 seconds
- Chapter personalization: < 10 seconds
- Urdu translation: < 8 seconds
- Embedding pipeline: < 10 minutes for 50 chapters
- Concurrent users: 50+ without degradation

**Constraints**:
- Free tier limits: Qdrant 1GB storage, Neon 0.5GB Postgres, OpenAI 500 req/min
- No external auth providers (Better Auth email/password only, no API keys)
- System must function fully without Neon Postgres (in-memory fallback)
- Code blocks must remain unchanged in Urdu translation
- API keys never exposed in client code
- HTTP-only cookies for session tokens (XSS prevention)

**Scale/Scope**:
- 50+ book chapters (Module 1-4: ROS2, Gazebo, Isaac Sim, VLA)
- ~250 embedded chunks (5 chunks/chapter average)
- 1,000 estimated users (free tier)
- 10 queries/minute per user rate limit
- 50KB max personalized content, 100KB max translated content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Content-First Development ✅

**Compliance**: All book content in `docs/` already exists and is structured progressively (Module 1 → Module 4). This feature enhances content accessibility without modifying source material. Personalization and translation preserve original content integrity.

**Evidence**:
- Book chapters complete in `docs/module-{1,2,3,4}/`
- RAG chatbot retrieves from existing content only (no external knowledge bases)
- Original content remains primary source of truth

### II. Docusaurus Framework Adherence ✅

**Compliance**: ChatWithBook component integrates via standard Docusaurus patterns (MDX components, theme swizzling, React Context API). No framework violations. Floating widget injected through `Root.tsx` swizzling per Docusaurus best practices.

**Evidence**:
- Component: `src/components/ChatWithBook.tsx` (standard location)
- Theme swizzle: `src/theme/Root.tsx` (documented Docusaurus pattern)
- Dedicated page: `src/pages/ai-chatbot.mdx` (standard MDX page)
- docusaurus.config.js updated for navbar link only

### III. GitHub Pages Deployment Readiness ✅

**Compliance**: Docusaurus frontend builds to static assets deployable to GitHub Pages. Backend deployed separately (e.g., Vercel, Render, Railway). CORS configured to allow GitHub Pages origin. Base URL configuration supports GitHub Pages path (`/Physical-ai-human-robotics-book/`).

**Evidence**:
- Frontend: `npm run build` generates static site in `build/`
- Backend: Containerized FastAPI for platform-agnostic deployment
- CORS: Configurable via `CORS_ORIGINS` env variable

**Deployment Separation**: Backend requires separate hosting (not GitHub Pages) due to dynamic API requirements. This is standard for Jamstack architectures.

### IV. Progressive Content Structure ✅

**Compliance**: Book structure unchanged. Chatbot citations reference specific chapters/sections, reinforcing progressive learning. Personalization adapts complexity level but maintains module sequence.

**Evidence**:
- Chatbot citations include `chapter_id`, `section`, `module_name`
- Personalization preserves chapter order, adjusts explanation depth only
- Translation maintains heading hierarchy and internal links

### V. Simplicity and Clarity ✅

**Compliance**: Core functionality (RAG chatbot) uses established libraries (OpenAI SDK, Qdrant). No custom LLM training or complex ML pipelines. MCP Server adds abstraction layer but simplifies external service management. Authentication via Better Auth (no custom auth implementation).

**Evidence**:
- OpenAI Assistants API handles conversation state (no custom context management)
- Qdrant Cloud managed service (no self-hosted vector DB)
- Better Auth library (no custom password hashing/session management)

**Complexity Justified**: MCP Server proxy layer required to centralize API key management and enable graceful degradation (spec requirement: "must function without Neon Postgres").

### VI. Version Control and Traceability ✅

**Compliance**: All code changes tracked in Git. API contracts versioned via OpenAPI spec. Data model schema versioned in `data-model.md`. Database migrations tracked via SQLAlchemy.

**Evidence**:
- Spec: `specs/001-rag-chatbot/spec.md` (requirements versioned)
- Plan: `specs/001-rag-chatbot/plan.md` (architecture versioned)
- Contracts: `specs/001-rag-chatbot/contracts/api-spec.yaml` (API v1.0.0)
- Database migrations: Versioned SQL scripts in `backend/migrations/`

---

**Constitution Re-Check (Post-Design)**: ✅ **PASSED**

All principles satisfied. Complexity justified by spec requirements (personalization, translation, graceful degradation). No unnecessary abstractions added.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── spec.md                        # Feature specification (/sp.specify output)
├── plan.md                        # This file (/sp.plan output)
├── research.md                    # Technology decisions (Phase 0 output)
├── data-model.md                  # Entity definitions (Phase 1 output)
├── quickstart.md                  # Local development guide (Phase 1 output)
├── contracts/
│   └── api-spec.yaml              # OpenAPI 3.0 REST API contract
├── checklists/
│   └── requirements.md            # Spec quality validation
└── tasks.md                       # Implementation tasks (/sp.tasks output - NOT YET CREATED)
```

### Source Code (repository root)

```text
backend/                           # FastAPI backend (NEW)
├── src/
│   ├── models/
│   │   ├── user.py               # User, UserProfile entities
│   │   ├── conversation.py       # Conversation, Message entities
│   │   ├── content.py            # PersonalizedContent, TranslatedContent
│   │   └── chunk.py              # DocumentChunk (Qdrant model)
│   ├── services/
│   │   ├── auth_service.py       # Better Auth integration
│   │   ├── rag_service.py        # Vector search + OpenAI Assistants
│   │   ├── personalization_service.py  # Background-based content generation
│   │   ├── translation_service.py      # Urdu translation
│   │   └── embedding_service.py        # Chunking + embedding pipeline
│   ├── api/
│   │   ├── auth.py               # /api/auth/* endpoints
│   │   ├── chat.py               # /api/chat/* endpoints
│   │   └── content.py            # /api/content/* endpoints
│   ├── mcp/
│   │   ├── server.py             # MCP Server middleware
│   │   ├── qdrant_proxy.py       # Qdrant API proxy
│   │   ├── neon_proxy.py         # Neon Postgres proxy
│   │   └── openai_proxy.py       # OpenAI API proxy
│   ├── database.py               # SQLAlchemy engine + session
│   ├── config.py                 # Pydantic Settings (env vars)
│   └── main.py                   # FastAPI app entry point
├── tests/
│   ├── unit/
│   │   ├── test_auth_service.py
│   │   ├── test_rag_service.py
│   │   ├── test_personalization_service.py
│   │   └── test_translation_service.py
│   ├── integration/
│   │   ├── test_auth_flow.py
│   │   ├── test_chat_flow.py
│   │   └── test_personalization_flow.py
│   └── contract/
│       └── test_api_spec.py      # Validate OpenAPI contract
├── scripts/
│   ├── init_db.py                # Initialize Neon Postgres schema
│   ├── init_qdrant.py            # Create Qdrant collection
│   └── embed_book_content.py     # Embedding pipeline script
├── migrations/                   # Database migration scripts
│   └── 001_initial_schema.sql
├── requirements.txt              # Python dependencies
├── .env.example                  # Environment variables template
├── Dockerfile                    # Backend containerization
└── README.md                     # Backend setup guide

src/                              # Docusaurus React components (ENHANCED)
├── components/
│   ├── ChatWithBook.tsx          # Main chatbot UI component (NEW)
│   ├── ChatWithBook.css          # Chatbot styling (NEW)
│   ├── ChatMessage.tsx           # Individual message component (NEW)
│   ├── CitationList.tsx          # Book citation display (NEW)
│   └── AuthModal.tsx             # Signup/signin modal (NEW)
└── theme/
    └── Root.tsx                  # Swizzled root to inject floating widget (NEW)

src/pages/                        # Docusaurus pages (ENHANCED)
├── ai-chatbot.mdx                # Dedicated chatbot page (NEW)
└── index.tsx                     # Homepage (existing)

docs/                             # Book content (EXISTING, unchanged)
├── module-1-ros2/
│   ├── index.md
│   ├── 01-ros2-basics-first-node.md
│   ├── 02-topics-services.md
│   └── 03-urdf-humanoids.md
├── module-2-gazebo/
│   ├── index.md
│   ├── 01-simulation-essentials.md
│   └── 02-sensors-ros2.md
├── module-3-isaac/
│   ├── index.md
│   ├── 01-isaac-sim-essentials.md
│   ├── 02-synthetic-data-perception.md
│   ├── 03-vslam.md
│   └── 04-navigation-humanoids.md
├── module-4-vla/
│   ├── index.md
│   ├── 01-vla-voice-to-action.md
│   ├── 02-llm-planning.md
│   └── 03-integration-case-studies.md
├── intro.md
├── quick-start.md
└── glossary.md

scripts/                          # Utility scripts (NEW)
└── embed_book_content.py         # Same as backend/scripts, for convenience

docusaurus.config.js              # Docusaurus config (ENHANCED)
package.json                      # Frontend dependencies (ENHANCED)
.env                              # Frontend env vars (API URL) (NEW)
```

**Structure Decision**: Web application architecture selected (backend + frontend). Backend is self-contained FastAPI project with MCP Server integrated as middleware. Frontend integrates into existing Docusaurus site via components and theme swizzling. This structure:
- Maintains existing book content structure (no disruption)
- Separates concerns (backend API, frontend UI)
- Supports independent deployment (backend to cloud, frontend to GitHub Pages)
- Enables local development with `npm run start` (frontend) + `uvicorn` (backend)

## Complexity Tracking

**No constitutional violations requiring justification.** All complexity is aligned with spec requirements and constitution principles.

Optional complexity justifications:

| Component | Complexity Added | Justification | Alternative Rejected |
|-----------|------------------|---------------|---------------------|
| MCP Server Layer | Adds proxy abstraction between backend and external services | Spec requires "all external services accessed via MCP Server at Context Level 7" + enables graceful degradation when services unavailable | Direct service calls would require error handling duplication across codebase and cannot support fallback strategies |
| In-Memory Cache | Dual storage (Neon Postgres + in-memory) for personalized/translated content | Spec requires "system must function without SQL storage" + performance optimization (1-hour TTL avoids redundant LLM calls) | Database-only storage violates graceful degradation requirement; regeneration-on-every-request exceeds 10-second performance goal |
| Better Auth Integration | Third-party auth library vs custom auth | Spec requires "no API keys required" which eliminates OAuth providers; Better Auth provides secure password hashing, session management, and database integration out-of-box | Custom auth implementation requires cryptography expertise, introduces security risks, and delays delivery |

## Phase 0: Research Outcomes

**All technical decisions resolved in [research.md](./research.md).**

Key decisions:
1. **RAG Architecture**: OpenAI Assistants API with function calling (vs. LangChain or custom pipeline)
2. **Embeddings**: text-embedding-3-small (1536d, cost-optimized for free tier)
3. **Auth**: Better Auth with email/password provider (no external API keys)
4. **Personalization**: GPT-4 Turbo with dynamic system prompts per user background
5. **Translation**: GPT-4 Turbo with custom prompt to preserve code blocks
6. **MCP Server**: FastAPI middleware acting as unified service proxy
7. **Caching**: In-memory LRU cache (100 personalized entries, 50 translated entries, 1-hour TTL)
8. **Chunking**: Fixed-size 800 tokens with 200 token overlap, preserving Markdown structure
9. **Database**: Neon Serverless Postgres with graceful fallback to in-memory session storage
10. **Docusaurus Integration**: Theme swizzling (`Root.tsx`) for floating widget

## Phase 1: Design Artifacts

### Data Model

**Complete entity definitions in [data-model.md](./data-model.md).**

Core entities:
- **User**: Authentication + background profile (software_bg, hardware_bg)
- **Conversation**: Chat session (user-linked or anonymous)
- **Message**: User query or assistant response with citations
- **DocumentChunk**: Embedded book content (Qdrant)
- **PersonalizedContent**: Cached user-specific chapter versions
- **TranslatedContent**: Cached Urdu translations
- **SelectedContext**: Ephemeral user text selections
- **EmbeddingMetadata**: Chunk enrichment data

Database schema: Neon Postgres (users, conversations, messages, personalized_content, translated_content)
Vector schema: Qdrant collection `book_content` (1536-dimensional vectors, cosine similarity)

### API Contracts

**Complete OpenAPI 3.0 specification in [contracts/api-spec.yaml](./contracts/api-spec.yaml).**

Endpoints:
- **Auth**: `/api/auth/signup`, `/api/auth/signin`, `/api/auth/signout`, `/api/auth/me`
- **Chat**: `/api/chat/conversations` (GET/POST), `/api/chat/conversations/{id}/messages` (GET/POST)
- **Content**: `/api/content/chapters/{id}/personalize`, `/api/content/chapters/{id}/translate`, `/api/content/chapters/{id}/original`
- **Admin**: `/api/admin/embeddings/pipeline`, `/api/admin/embeddings/status`

### Quickstart Guide

**Complete local development guide in [quickstart.md](./quickstart.md).**

Covers:
- Environment setup (Python venv, Node.js, API keys)
- Database initialization (Neon Postgres schema)
- Qdrant collection setup
- Embedding pipeline execution
- FastAPI backend startup
- Docusaurus frontend startup
- Testing core features (chatbot, signup, personalization, translation)
- Troubleshooting common issues

## Implementation Phases (Next Steps)

**Phase 2** (via `/sp.tasks` command):
1. Backend foundation: FastAPI app skeleton, database connection, Qdrant client
2. Authentication: Better Auth integration, signup/signin endpoints
3. Embedding pipeline: Markdown chunking, embedding generation, Qdrant upload
4. RAG service: Vector search, OpenAI Assistants API integration, citation extraction
5. Personalization service: Background-aware system prompts, content regeneration
6. Translation service: Urdu translation with code block preservation
7. MCP Server: Service proxy layer with error handling and fallback logic
8. Frontend components: ChatWithBook UI, AuthModal, message rendering
9. Docusaurus integration: Theme swizzling, dedicated chatbot page, navbar link
10. Testing: Unit tests, integration tests, API contract validation
11. Deployment: Dockerize backend, configure CORS, deploy to hosting platform

**Phase 3** (Manual/Future):
- Production deployment
- Monitoring and observability
- User feedback collection
- Content update automation (re-embedding on chapter changes)
- Performance optimization (caching strategies, CDN for embeddings)

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| OpenAI rate limits exceeded | Medium | High | Implement exponential backoff, queue requests, add rate limit monitoring |
| Qdrant Cloud outage | Low | High | Return graceful error to user, implement health check endpoint |
| Neon Postgres unavailable | Low | Medium | Automatic fallback to in-memory storage (already designed) |
| Urdu translation inaccuracy | Medium | Medium | Back-translation validation, community review, iterative prompt improvement |
| Code blocks modified in translation | Low | High | Regex validation post-translation, reject if code changed, alert user |
| Free tier limits reached | High | Medium | Monitor usage, implement per-user rate limiting (10 queries/min), queue non-critical requests |
| Better Auth integration issues | Medium | High | Fallback to anonymous chatbot, prioritize auth in testing, verify library compatibility |
| Personalization doesn't match user background | Medium | Medium | User feedback collection, A/B testing prompts, manual review of generated content |

## Success Metrics (From Spec)

Tracking alignment with Success Criteria (SC-001 to SC-017):

- **SC-001**: Response time < 5 seconds → Measure via backend logging, p95 latency
- **SC-002**: 90% responses include citations → Count citations in Message.citations field
- **SC-003**: Widget load < 2 seconds → Frontend performance profiling
- **SC-004**: 100% embedding pipeline success → Script output validation
- **SC-005**: System works without Neon Postgres → Manual testing with DATABASE_URL unset
- **SC-006**: Conversation persists across 5 page navigations → E2E test with React Context
- **SC-007**: 30% relevance improvement with selected text → A/B testing (with/without context)
- **SC-008**: Cross-browser compatibility → Manual testing on Chrome, Firefox, Safari, Edge
- **SC-009**: 100% MCP Server usage → Audit code for direct service API calls (should be zero)
- **SC-010**: Zero exposed API keys → Static analysis + code review
- **SC-011**: Signup < 2 minutes → User testing, form usability
- **SC-012**: 80% user satisfaction with personalization → Post-interaction survey
- **SC-013**: 100% code preservation in Urdu → Automated regex validation
- **SC-014**: Personalization < 10 seconds → Backend timing logs
- **SC-015**: Session history accessible → Database query testing
- **SC-016**: Anonymous chatbot works without auth → Manual testing, logout scenario
- **SC-017**: Urdu semantic accuracy → Back-translation comparison, manual review

## Next Steps

1. ✅ **Phase 0 Complete**: Technology research and decisions documented
2. ✅ **Phase 1 Complete**: Data model, API contracts, quickstart guide created
3. **Pending**: Run `/sp.tasks` command to generate implementation task breakdown
4. **Pending**: Implement tasks sequentially (auth → embedding → RAG → personalization → translation → frontend)
5. **Pending**: Deploy to production and monitor metrics

---

**Plan Status**: **COMPLETE** - Ready for task generation via `/sp.tasks`

**Last Updated**: 2025-12-10
**Author**: Development Team via `/sp.plan` command
