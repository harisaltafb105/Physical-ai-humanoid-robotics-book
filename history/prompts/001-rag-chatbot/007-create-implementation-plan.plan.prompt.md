---
id: 007
title: Create RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: harisaltafb105
command: /sp.plan
labels: ["rag", "chatbot", "planning", "architecture", "fastapi", "docusaurus", "openai", "qdrant", "neon", "better-auth"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/plan.md
  - specs/001-rag-chatbot/research.md
  - specs/001-rag-chatbot/data-model.md
  - specs/001-rag-chatbot/contracts/api-spec.yaml
  - specs/001-rag-chatbot/quickstart.md
tests:
  - Constitution check passed (all 6 principles validated)
  - Research decisions documented for 10 technical areas
  - Data model validated (8 entities, Postgres + Qdrant schemas)
  - API contract complete (OpenAPI 3.0 spec with 13 endpoints)
---

## Prompt

No explicit prompt arguments provided - `/sp.plan` command executed on `001-rag-chatbot` branch.

**Context**: Create comprehensive implementation plan for RAG chatbot feature with authentication, personalization, and Urdu translation based on updated specification.

## Response snapshot

Created complete implementation plan with Phase 0 (Research) and Phase 1 (Design) artifacts for RAG chatbot system.

**Phase 0 - Research Outcomes (research.md):**

Resolved 10 critical technical decisions:
1. **RAG Architecture**: OpenAI Assistants API with function calling (over LangChain or ChatKit SDK)
2. **Embeddings**: text-embedding-3-small (1536d, cost-optimized for Qdrant free tier)
3. **Authentication**: Better Auth with email/password (no external API keys required)
4. **Personalization Engine**: GPT-4 Turbo with dynamic system prompts based on user background (beginner/intermediate/advanced × simulation-only/hobbyist/professional)
5. **Urdu Translation**: GPT-4 Turbo with custom prompts preserving code blocks and technical terms
6. **MCP Server Integration**: FastAPI middleware as unified proxy for Qdrant, Neon, and OpenAI
7. **Docusaurus Integration**: Theme swizzling (`Root.tsx`) for floating widget injection
8. **Neon Postgres Schema**: Minimal relational schema with graceful fallback to in-memory storage
9. **Chunking Pipeline**: 800 tokens/chunk with 200 token overlap, preserving Markdown structure
10. **Error Handling**: Layered fallback strategy (Better Auth unavailable → anonymous chatbot, Neon unavailable → in-memory sessions)

**Phase 1 - Design Artifacts Created:**

1. **Data Model (data-model.md)**:
   - 8 entities defined: User, Conversation, Message, DocumentChunk, PersonalizedContent, TranslatedContent, SelectedContext, EmbeddingMetadata
   - Complete Postgres schema (users, conversations, messages, personalized_content, translated_content tables)
   - Qdrant collection schema (book_content with 1536-dimensional vectors, cosine similarity)
   - Storage estimates: 52.5MB total (well within free tier limits)

2. **API Contracts (contracts/api-spec.yaml)**:
   - OpenAPI 3.0 specification with 13 endpoints across 4 categories
   - Auth endpoints: signup, signin, signout, /me
   - Chat endpoints: conversations CRUD, messages streaming
   - Content endpoints: personalize, translate, original
   - Admin endpoints: embedding pipeline trigger and status
   - Request/response schemas, error handling, authentication schemes

3. **Quickstart Guide (quickstart.md)**:
   - Complete local development setup (Python venv, Node.js, API keys)
   - Step-by-step initialization (database schema, Qdrant collection, embedding pipeline)
   - Full-stack startup instructions (FastAPI backend + Docusaurus frontend)
   - Feature testing workflows (anonymous chat, signup/signin, personalization, Urdu translation)
   - Troubleshooting common issues (DB connection, Qdrant auth, OpenAI rate limits)

4. **Implementation Plan (plan.md)**:
   - Technical context: Python 3.11+, TypeScript 5.x, FastAPI, Docusaurus 3.0+, OpenAI GPT-4 Turbo
   - Constitution check: All 6 principles validated (Content-First, Docusaurus Adherence, GitHub Pages Readiness, Progressive Structure, Simplicity, Version Control)
   - Project structure: Web application (backend/ + enhanced src/ directories)
   - Complexity tracking: MCP Server layer, in-memory cache, Better Auth integration (all justified by spec requirements)
   - Performance goals: <5s chatbot response, <3s auth, <10s personalization, <8s translation
   - Risk analysis: 8 risks identified with mitigation strategies
   - Success metrics: Mapped all 17 spec success criteria to testing/validation approaches

**Key Architectural Decisions:**

- **Graceful Degradation**: System functions without Neon Postgres (in-memory fallback) and without Better Auth (anonymous chatbot access)
- **MCP Server Layer**: Centralized proxy for all external services (Qdrant, Neon, OpenAI) enabling unified error handling and rate limiting
- **Dual Caching Strategy**: In-memory LRU cache (primary) with optional Neon Postgres persistence for personalized/translated content
- **Separation of Concerns**: Backend deployed independently from Docusaurus frontend (Jamstack architecture)

**Constitution Compliance:**
All 6 principles passed:
- Content-First: Book content unchanged, chatbot enhances accessibility
- Docusaurus Adherence: Standard component patterns, theme swizzling per docs
- GitHub Pages Ready: Frontend static build, backend separately hosted
- Progressive Structure: Citations reinforce module sequence
- Simplicity: Established libraries (OpenAI, Qdrant, Better Auth), no custom ML
- Version Control: Git tracking, OpenAPI versioning, data model documentation

## Outcome

- ✅ Impact: Comprehensive architectural plan ready for task generation
- 🧪 Tests: Constitution check (6/6 passed), data model validation, API contract completeness
- 📁 Files: plan.md, research.md, data-model.md, api-spec.yaml, quickstart.md
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation task breakdown for 11 work streams (backend foundation, auth, embedding, RAG, personalization, translation, MCP, frontend, integration, testing, deployment)
- 🧠 Reflection: Complex feature successfully decomposed into clear phases. MCP Server abstraction enables testability and graceful degradation. Free-tier optimization prioritized throughout (in-memory cache, shared translation cache, Qdrant/Neon limits considered).

## Evaluation notes (flywheel)

- Failure modes observed: None - planning phase completed successfully with all artifacts generated
- Graders run and results (PASS/FAIL): PASS - Constitution check validated, research decisions documented, data model complete, API contracts defined
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): During implementation, validate Better Auth compatibility with FastAPI (library documentation review needed); consider alternative: custom JWT auth if Better Auth integration proves complex
