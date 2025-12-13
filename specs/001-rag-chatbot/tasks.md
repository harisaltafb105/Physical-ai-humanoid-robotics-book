# Implementation Tasks: RAG Chatbot with Auth, Personalization & Translation

**Feature**: 001-rag-chatbot
**Branch**: `001-rag-chatbot`
**Generated**: 2025-12-10
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Task Summary

- **Total Tasks**: 89
- **User Stories**: 8 (5× P1, 1× P2, 2× P3)
- **Parallel Opportunities**: 47 parallelizable tasks
- **MVP Scope**: User Stories 1-2 (Core RAG chatbot with selected text context)

## Task Format

```
- [ ] [TaskID] [P?] [Story?] Description with file path
```

- **TaskID**: T001, T002, etc. (sequential execution order)
- **[P]**: Parallelizable (can run concurrently with other [P] tasks)
- **[Story]**: User story label ([US1], [US2], etc.) for story-specific tasks

---

## Phase 1: Setup & Project Initialization

**Goal**: Bootstrap project structure, dependencies, and development environment

**Tasks**:

- [X] T001 Create backend directory structure: `backend/{src/{models,services,api,mcp},tests/{unit,integration,contract},scripts,migrations}/`
- [X] T002 [P] Create `backend/requirements.txt` with dependencies: FastAPI 0.109+, OpenAI SDK 1.6+, Qdrant Client 1.7+, SQLAlchemy 2.0+, psycopg2 2.9+, python-dotenv, bcrypt, python-jose
- [X] T003 [P] Create `backend/.env.example` template with placeholders for OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, AUTH_SECRET, CORS_ORIGINS
- [X] T004 [P] Create `backend/src/config.py` using Pydantic Settings to load environment variables
- [X] T005 [P] Create `backend/src/database.py` with SQLAlchemy engine, session factory, and Base model
- [X] T006 [P] Create `backend/Dockerfile` for containerized deployment
- [X] T007 [P] Create `backend/README.md` with setup instructions
- [X] T008 [P] Create `backend/scripts/init_db.py` to initialize Neon Postgres schema from data-model.md
- [X] T009 [P] Create `backend/scripts/init_qdrant.py` to create Qdrant collection with 1536-dimensional vectors (cosine distance)
- [X] T010 Install frontend dependencies: `npm install` (verify Docusaurus 3.0+ and React 18+)
- [X] T011 [P] Create `src/components/` directory for React components
- [X] T012 [P] Create `.env` file in project root for frontend (API_URL=http://localhost:8000)

**Validation**: Run `python backend/src/config.py` to verify env loading; verify directory structure matches plan.md

---

## Phase 2: Foundational Infrastructure (Blocking Prerequisites)

**Goal**: Build MCP Server proxy layer and embedding pipeline (blocks all user stories)

**Tasks**:

- [X] T013 Create `backend/src/mcp/server.py` with FastAPI middleware class for unified service proxy
- [X] T014 [P] Create `backend/src/mcp/qdrant_proxy.py` with methods: `search_vectors(query_vector, limit)`, `upsert_vectors(points)`, `get_collection_info()`
- [X] T015 [P] Create `backend/src/mcp/neon_proxy.py` with methods: `execute_query(sql, params)`, `check_connection()`, fallback to in-memory storage on failure
- [X] T016 [P] Create `backend/src/mcp/openai_proxy.py` with methods: `create_embedding(text)`, `chat_completion(messages)`, `create_assistant()`, exponential backoff for rate limits
- [X] T017 Integrate MCP Server middleware into `backend/src/main.py` FastAPI app
- [X] T018 Create `backend/src/services/embedding_service.py` with `chunk_markdown(file_path)` method: parse frontmatter, split by headings, create 800-token chunks with 200-token overlap
- [X] T019 Add `create_embeddings(chunks)` method to embedding_service.py using MCP OpenAI proxy
- [X] T020 Add `upload_to_qdrant(chunks, embeddings, metadata)` method to embedding_service.py using MCP Qdrant proxy
- [X] T021 Create `backend/scripts/embed_book_content.py` CLI script: processes docs/**/*.md files, detects changes via file hashes, runs embedding pipeline
- [X] T022 Create `backend/migrations/001_initial_schema.sql` with tables: users, conversations, messages, personalized_content, translated_content (from data-model.md)
- [ ] T023 Test embedding pipeline: Run `python backend/scripts/embed_book_content.py` on sample chapter, verify Qdrant upload via MCP Server

**Validation**: MCP Server proxies all external service calls; embedding pipeline processes 1 chapter successfully; 0 direct Qdrant/Neon/OpenAI client calls in codebase (all via MCP)

---

## Phase 3: User Story 1 - Ask Questions About Book Content (P1)

**Story Goal**: Enable readers to ask questions and receive answers with citations from book content

**Independent Test**: Ask "What is ROS2?" → verify response includes Module 1 content with chapter citations

**Tasks**:

- [X] T024 [US1] Create `backend/src/models/conversation.py` with Conversation and Message SQLAlchemy models (from data-model.md schema)
- [X] T025 [P] [US1] Create `backend/src/models/chunk.py` with DocumentChunk Pydantic model for Qdrant payload schema
- [X] T026 [US1] Create `backend/src/services/rag_service.py` with `search_book_content(query, limit=5)` method: creates embedding via MCP, searches Qdrant, returns top chunks
- [X] T027 [US1] Add `generate_response(query, retrieved_chunks, conversation_history)` method to rag_service.py: uses OpenAI Assistants API via MCP with function calling
- [X] T028 [US1] Add `extract_citations(chunks)` method to rag_service.py: formats chapter_id, section, relevance_score for Message.citations JSONB
- [X] T029 [US1] Create `backend/src/api/chat.py` with POST `/api/chat/conversations` endpoint: creates new Conversation, returns conversation_id
- [X] T030 [US1] Add POST `/api/chat/conversations/{id}/messages` endpoint to chat.py: accepts user query, calls rag_service, stores user+assistant messages, returns both with citations
- [X] T031 [US1] Add GET `/api/chat/conversations/{id}/messages` endpoint to chat.py: retrieves all messages for conversation
- [X] T032 [US1] Add GET `/api/chat/conversations` endpoint to chat.py: lists user conversations (returns empty array for anonymous)
- [X] T033 [US1] Create `src/components/ChatMessage.tsx` React component: displays single message (user or assistant) with timestamp
- [X] T034 [P] [US1] Create `src/components/CitationList.tsx` React component: renders citations as clickable links to book chapters
- [X] T035 [US1] Create `src/components/ChatWithBook.tsx` main component: chat UI with input field, message list (using ChatMessage), send button, minimize/maximize controls
- [X] T036 [US1] Implement `sendMessage(query)` function in ChatWithBook.tsx: POST to `/api/chat/conversations/{id}/messages`, append messages to state
- [X] T037 [US1] Implement conversation state management in ChatWithBook.tsx using React useState: messages array, conversation_id, loading state
- [X] T038 [US1] Add error handling in ChatWithBook.tsx: display user-friendly errors for rate limits, service unavailable, network issues
- [X] T039 [US1] Create `src/pages/ai-chatbot.mdx` dedicated page: imports ChatWithBook component with fullScreen prop
- [X] T040 [US1] Update `docusaurus.config.js`: add "AI Chatbot" navbar link pointing to `/ai-chatbot`

**Integration Test (US1)**:
- [ ] T041 [US1] Manual test: Start backend + frontend, ask "What is VSLAM?" on dedicated page, verify response includes Module 3 citations and displays in UI

**Validation**: Anonymous user can ask questions → receive answers with book citations → citations link to correct chapters

---

## Phase 4: User Story 2 - Query with Selected Text Context (P1)

**Story Goal**: Allow users to select text and ask context-aware questions

**Independent Test**: Select code snippet → ask "Explain this code" → verify response references selected text

**Tasks**:

- [X] T042 [US2] Add `selected_context` optional parameter to POST `/api/chat/conversations/{id}/messages` endpoint schema: `{text, source_url, chapter_id}`
- [X] T043 [US2] Update `generate_response()` in rag_service.py: prepend selected_context to system prompt if provided
- [X] T044 [US2] Create `src/components/TextSelectionHandler.tsx`: detects text selection via `window.getSelection()`, stores in React Context
- [X] T045 [US2] Add "Ask about this" button that appears on text selection in TextSelectionHandler.tsx
- [X] T046 [US2] Update ChatWithBook.tsx to consume TextSelectionHandler context: include selected text in API request if present
- [X] T047 [US2] Implement selected text truncation in ChatWithBook.tsx: max 2000 chars, notify user if exceeded
- [X] T048 [US2] Add selected text display in ChatWithBook.tsx: show highlighted text above input field when present, allow clear/dismiss

**Integration Test (US2)**:
- [ ] T049 [US2] Manual test: Select URDF code from Module 1 → click "Ask about this" → ask "What does this do?" → verify response explains selected code

**Validation**: User selects text → chatbot response directly references selection → selection persists across navigation for session duration

---

## Phase 5: User Story 3 - Sign Up and Sign In with Better Auth (P1)

**Story Goal**: Enable user authentication with background profile collection

**Independent Test**: Complete signup with email/password/backgrounds → sign out → sign in → verify profile loaded

**Tasks**:

- [ ] T050 [US3] Create `backend/src/models/user.py` with User SQLAlchemy model: id, email, password_hash, software_background (enum), hardware_background (enum), created_at, updated_at
- [ ] T051 [US3] Create `backend/src/services/auth_service.py` with `create_user(email, password, software_bg, hardware_bg)` method: validates inputs, hashes password with bcrypt, inserts via MCP Neon proxy
- [ ] T052 [US3] Add `authenticate_user(email, password)` method to auth_service.py: retrieves user, verifies bcrypt hash, returns user object or None
- [ ] T053 [US3] Add `create_session_token(user_id)` method to auth_service.py: generates JWT with user_id + expiration (7 days), uses AUTH_SECRET from config
- [ ] T054 [US3] Add `verify_session_token(token)` method to auth_service.py: decodes JWT, returns user_id or raises exception
- [ ] T055 [US3] Create `backend/src/api/auth.py` with POST `/api/auth/signup` endpoint: validates email format, checks uniqueness, calls auth_service.create_user(), sets HTTP-only session cookie
- [ ] T056 [US3] Add POST `/api/auth/signin` endpoint to auth.py: calls auth_service.authenticate_user(), sets session cookie on success, returns 401 on failure
- [ ] T057 [US3] Add POST `/api/auth/signout` endpoint to auth.py: clears session cookie (Max-Age=0)
- [ ] T058 [US3] Add GET `/api/auth/me` endpoint to auth.py: verifies session token, returns user profile (email, software_bg, hardware_bg)
- [ ] T059 [US3] Create auth middleware in `backend/src/api/auth.py`: decorator `@require_auth` that verifies session token, attaches user to request
- [ ] T060 [US3] Create `src/components/AuthModal.tsx` React component: tabs for "Sign Up" and "Sign In", form fields for email/password
- [ ] T061 [US3] Add background selection dropdowns to signup tab in AuthModal.tsx: software (beginner/intermediate/advanced), hardware (simulation-only/hobbyist/professional)
- [ ] T062 [US3] Implement signup submit handler in AuthModal.tsx: POST to `/api/auth/signup`, store user state in React Context
- [ ] T063 [US3] Implement signin submit handler in AuthModal.tsx: POST to `/api/auth/signin`, store user state
- [ ] T064 [US3] Create `src/context/AuthContext.tsx`: provides user state, signin/signup/signout functions to app
- [ ] T065 [US3] Wrap ChatWithBook component with AuthContext.Provider in `src/theme/Root.tsx`
- [ ] T066 [US3] Add "Sign In / Sign Up" button to ChatWithBook.tsx header when user not authenticated
- [ ] T067 [US3] Add user profile display to ChatWithBook.tsx header when authenticated: email, background badges, "Sign Out" button

**Integration Test (US3)**:
- [ ] T068 [US3] Manual test: Click "Sign Up" → fill form (test@example.com, password, beginner, hobbyist) → verify account created → sign out → sign in → verify profile displays

**Validation**: Users can create accounts → sign in securely → session persists across browser restarts (within 7-day expiration) → profile data stored in Neon Postgres

---

## Phase 6: User Story 4 - Personalize Chapter Content Based on Background (P1)

**Story Goal**: Regenerate chapter content tailored to user's software/hardware background

**Independent Test**: Sign in as beginner/hobbyist → click "Personalize" on ROS2 chapter → verify simplified explanations generated

**Tasks**:

- [ ] T069 [US4] Create `backend/src/models/content.py` with PersonalizedContent SQLAlchemy model: id, user_id, chapter_id, software_background, hardware_background, personalized_content (Text), created_at, expires_at
- [ ] T070 [US4] Create `backend/src/services/personalization_service.py` with `generate_system_prompt(software_bg, hardware_bg)` method: returns tailored prompt ("Explain for beginners...", "Focus on best practices for advanced...")
- [ ] T071 [US4] Add `personalize_chapter(chapter_id, user)` method to personalization_service.py: fetches original markdown, constructs system prompt, calls OpenAI via MCP, returns personalized markdown
- [ ] T072 [US4] Add in-memory LRU cache to personalization_service.py: key `personalized_{user_id}_{chapter_id}`, max 100 entries, 1-hour TTL
- [ ] T073 [US4] Add cache check/set logic in `personalize_chapter()`: return cached if exists and not expired, otherwise generate and cache
- [ ] T074 [US4] Create `backend/src/api/content.py` with POST `/api/content/chapters/{chapter_id}/personalize` endpoint (requires @require_auth): calls personalization_service, returns personalized markdown
- [ ] T075 [US4] Add GET `/api/content/chapters/{chapter_id}/original` endpoint to content.py: reads original markdown from docs/ directory
- [ ] T076 [US4] Create `src/components/PersonalizeButton.tsx`: button displayed at chapter start for authenticated users
- [ ] T077 [US4] Implement personalize handler in PersonalizeButton.tsx: POST to `/api/content/chapters/{id}/personalize`, replace chapter content in DOM
- [ ] T078 [US4] Add loading state to PersonalizeButton.tsx: show spinner during 10-second generation, disable button
- [ ] T079 [US4] Add "Reset to Original" button in PersonalizeButton.tsx: calls original endpoint, restores chapter content
- [ ] T080 [US4] Inject PersonalizeButton component into chapter pages via Docusaurus swizzling: update `src/theme/DocItem.tsx` (if needed) or use doc frontmatter

**Integration Test (US4)**:
- [ ] T081 [US4] Manual test: Sign in as beginner/simulation-only → navigate to Module 2 Gazebo chapter → click "Personalize" → verify content regenerates with simplified language and simulation focus

**Validation**: Logged-in users see "Personalize" button → personalized content generated within 10 seconds → content adapts to user background → original content restorable

---

## Phase 7: User Story 5 - Translate Chapter Content to Urdu (P1)

**Story Goal**: Translate chapter to Urdu while preserving code blocks and technical terms

**Independent Test**: Sign in → click "Translate to Urdu" → verify Urdu text with English code blocks

**Tasks**:

- [ ] T082 [US5] Create `backend/src/models/content.py` with TranslatedContent SQLAlchemy model: id, chapter_id, language ('ur'), translated_content (Text), code_blocks_preserved (Boolean), created_at, expires_at
- [ ] T083 [US5] Create `backend/src/services/translation_service.py` with `translate_to_urdu(chapter_content)` method: constructs prompt "Translate to Urdu, preserve code blocks...", calls OpenAI via MCP
- [ ] T084 [US5] Add code block preservation validation in translation_service.py: regex check that ` ```...``` ` blocks are unchanged, set code_blocks_preserved flag
- [ ] T085 [US5] Add shared translation cache to translation_service.py: key `translated_ur_{chapter_id}` (user-agnostic), max 50 entries, 1-hour TTL
- [ ] T086 [US5] Add POST `/api/content/chapters/{chapter_id}/translate` endpoint to content.py (requires @require_auth): accepts language='ur', calls translation_service, returns translated markdown
- [ ] T087 [US5] Create `src/components/TranslateButton.tsx`: "Translate to Urdu" button at chapter start for authenticated users
- [ ] T088 [US5] Implement translate handler in TranslateButton.tsx: POST to translate endpoint, replace chapter content, add "Back to English" toggle
- [ ] T089 [US5] Add language state to ChatWithBook.tsx: when chapter is in Urdu, accept Urdu queries and respond in Urdu (pass language hint to backend)
- [ ] T090 [US5] Update `/api/chat/conversations/{id}/messages` endpoint: accept optional `language` parameter, pass to OpenAI prompt as "Respond in Urdu if language=ur"
- [ ] T091 [US5] Add loading state to TranslateButton.tsx: show spinner during 8-second translation, disable button
- [ ] T092 [US5] Inject TranslateButton component into chapter pages alongside PersonalizeButton

**Integration Test (US5)**:
- [ ] T093 [US5] Manual test: Sign in → navigate to Module 1 ROS2 chapter → click "Translate to Urdu" → verify text is Urdu, code blocks unchanged → ask chatbot question in Urdu → verify Urdu response

**Validation**: Logged-in users see "Translate" button → translation completes in <8 seconds → code blocks remain English → chatbot supports Urdu queries when chapter translated

---

## Phase 8: User Story 6 - Persistent Chat via Floating Widget (P2)

**Story Goal**: Maintain conversation across page navigation with floating widget always accessible

**Independent Test**: Start chat on page 1 → navigate to page 2 → verify conversation persists and widget visible

**Tasks**:

- [ ] T094 [US6] Update ChatWithBook.tsx: add minimize/maximize state toggle
- [ ] T095 [US6] Add floating button to ChatWithBook.tsx: fixed position bottom-right, shows when minimized
- [ ] T096 [US6] Implement conversation persistence in ChatWithBook.tsx using sessionStorage: save messages array, conversation_id on every update
- [ ] T097 [US6] Add restore logic in ChatWithBook.tsx componentDidMount: load conversation from sessionStorage if exists
- [ ] T098 [US6] Update `src/theme/Root.tsx`: inject ChatWithBook as global component, not per-page
- [ ] T099 [US6] Add CSS for floating widget in `src/components/ChatWithBook.css`: z-index 1000, positioned fixed, responsive sizing
- [ ] T100 [US6] Add click-outside handler to ChatWithBook.tsx: minimize widget when clicking outside (using useRef + event listener)
- [ ] T101 [US6] Add independent tab state to ChatWithBook.tsx: each browser tab maintains separate sessionStorage conversation key

**Integration Test (US6)**:
- [ ] T102 [US6] Manual test: Open widget on Quick Start page → ask 2 questions → navigate to Module 2 → verify widget remains visible, conversation history intact → minimize → reopen → verify history persists

**Validation**: Widget visible on all pages → conversation persists across navigation → minimize/maximize works → each tab has independent conversation state

---

## Phase 9: User Story 7 - Access Dedicated Chatbot Page (P3)

**Story Goal**: Provide full-screen chat interface on dedicated page

**Independent Test**: Navigate to /ai-chatbot → verify full chat interface → conversations separate from floating widget

**Tasks**:

- [ ] T103 [US7] Update `src/pages/ai-chatbot.mdx`: import ChatWithBook with `fullScreen={true}` and `dedicatedPage={true}` props
- [ ] T104 [US7] Add fullScreen mode styling to ChatWithBook.tsx: full viewport height, no minimize button, centered layout
- [ ] T105 [US7] Add introduction message to dedicated page mode in ChatWithBook.tsx: "Welcome to the AI Chatbot! Ask any questions about ROS2, Gazebo, Isaac Sim, or VLA."
- [ ] T106 [US7] Use separate sessionStorage key for dedicated page conversations: `chatbot_dedicated_{conv_id}` vs `chatbot_widget_{conv_id}`

**Integration Test (US7)**:
- [ ] T107 [US7] Manual test: Navigate to /ai-chatbot → ask questions → verify full-screen interface → open floating widget on different page → verify separate conversation state

**Validation**: /ai-chatbot page accessible from navbar → full-screen UI functional → conversations independent from floating widget

---

## Phase 10: User Story 8 - Review Conversation History Across Sessions (P3)

**Story Goal**: Allow returning users to load previous conversations

**Independent Test**: Have conversation → close browser → return → verify conversation loadable from history

**Tasks**:

- [ ] T108 [US8] Update GET `/api/chat/conversations` endpoint: filter by user_id (from session token), order by updated_at DESC, return list with titles
- [ ] T109 [US8] Auto-generate conversation title in POST `/api/chat/conversations/{id}/messages`: set title to truncated first user message (max 50 chars)
- [ ] T110 [US8] Create `src/components/ConversationHistory.tsx`: displays list of past conversations with date/time, first query as title
- [ ] T111 [US8] Add "Load Conversation" button to ChatWithBook.tsx: opens ConversationHistory modal
- [ ] T112 [US8] Implement load conversation handler: GET `/api/chat/conversations/{id}/messages`, replace current messages state
- [ ] T113 [US8] Add graceful degradation in ConversationHistory.tsx: if Neon Postgres unavailable (no history), show message "Conversation history requires persistent storage"
- [ ] T114 [US8] Add delete conversation action to ConversationHistory.tsx (optional): DELETE `/api/chat/conversations/{id}`

**Integration Test (US8)**:
- [ ] T115 [US8] Manual test (with Neon Postgres): Have conversation → sign out → close browser → return → sign in → click "Load Conversation" → verify past conversations listed → load one → verify messages display

**Validation**: Logged-in users can view conversation history → load previous conversations → history unavailable when Neon Postgres down (graceful degradation)

---

## Phase 11: Polish & Cross-Cutting Concerns

**Goal**: Testing, documentation, deployment preparation

**Tasks**:

- [ ] T116 [P] Write unit tests for auth_service.py: test signup validation, password hashing, JWT creation/verification
- [ ] T117 [P] Write unit tests for rag_service.py: test search_book_content with mocked MCP proxy, citation extraction
- [ ] T118 [P] Write unit tests for personalization_service.py: test system prompt generation for different backgrounds
- [ ] T119 [P] Write unit tests for translation_service.py: test code block preservation validation
- [ ] T120 [P] Write integration test for chat flow: signup → ask question → verify response with citations
- [ ] T121 [P] Write integration test for personalization flow: signin → personalize chapter → verify content differs from original
- [ ] T122 [P] Write integration test for translation flow: signin → translate chapter → verify Urdu content, code preserved
- [ ] T123 [P] Create API contract test in `backend/tests/contract/test_api_spec.py`: validate all endpoints match api-spec.yaml OpenAPI schema
- [ ] T124 Create `backend/.dockerignore` file excluding .env, __pycache__, .pytest_cache
- [ ] T125 Add health check endpoint to `backend/src/main.py`: GET `/health` returns {status, services: {qdrant, database, openai}}
- [ ] T126 [P] Add CORS middleware to `backend/src/main.py`: allow origins from CORS_ORIGINS env var
- [ ] T127 [P] Add rate limiting middleware to `backend/src/main.py`: 10 requests/minute per user (using slowapi or custom)
- [ ] T128 [P] Create `backend/tests/README.md` documenting how to run tests: `pytest tests/ -v`
- [ ] T129 Update root `README.md`: add sections for Features, Architecture, Local Development, Deployment
- [ ] T130 Create deployment guide in `specs/001-rag-chatbot/deployment.md`: backend to Railway/Render, frontend to GitHub Pages, environment variables setup
- [ ] T131 Test full stack locally: backend + frontend + all user stories end-to-end
- [ ] T132 Create sample .env file with fake API keys for testing: `backend/.env.sample`
- [ ] T133 Add error boundary to ChatWithBook.tsx: catch React errors, display fallback UI
- [ ] T134 Add accessibility attributes to ChatWithBook.tsx: aria-labels, keyboard navigation for widget
- [ ] T135 Optimize ChatWithBook.css for mobile: responsive breakpoints, touch-friendly button sizes
- [ ] T136 Add logging to all backend services: use structlog for JSON logs, log levels per environment
- [ ] T137 [P] Create monitoring script: `backend/scripts/check_health.py` pings all services, reports status
- [ ] T138 [P] Document MCP Server architecture in `backend/src/mcp/README.md`: proxy pattern, fallback strategies
- [ ] T139 Run embedding pipeline on full book content: `python backend/scripts/embed_book_content.py --all`
- [ ] T140 Validate all success criteria from spec.md: response time <5s, 90% citation rate, 100% embedding success, etc.

**Validation**: All tests pass → API matches OpenAPI spec → Full stack runs locally → Documentation complete → Ready for deployment

---

## Dependencies & Execution Order

### User Story Dependencies

```
Phase 1 (Setup) → Phase 2 (Foundational)
                    ↓
        ┌───────────┼───────────┬───────────┬───────────┐
        ↓           ↓           ↓           ↓           ↓
      US1(P1)     US2(P1)     US3(P1)     US4(P1)     US5(P1)
        ↓           │           │           │           │
      US6(P2) ←─────┘           │           │           │
        ↓                       │           │           │
      US7(P3)                   │           │           │
        ↓                       │           │           │
      US8(P3) ←─────────────────┴───────────┴───────────┘
```

**Blocking Dependencies**:
- **Phase 2** blocks all user stories (must complete first)
- **US3** (Auth) blocks **US4** (Personalization), **US5** (Translation), **US8** (History)
- **US1** (Core Chat) blocks **US2** (Selected Context), **US6** (Floating Widget)
- **US6** can run after US1+US2 complete
- **US7** independent (can run anytime after US1)
- **US8** requires US3 (auth for user-specific history)

**Parallel Opportunities**:
- Within each phase: tasks marked [P] can run concurrently
- US1 and US3 can be developed in parallel after Phase 2
- US4 and US5 can be developed in parallel (both depend on US3)
- All Phase 11 test tasks can run in parallel

### Critical Path (Minimum MVP)

**MVP = US1 + US2** (Core RAG chatbot with selected text context)

1. Phase 1: Setup (T001-T012) - **1 day**
2. Phase 2: Foundational (T013-T023) - **2 days**
3. Phase 3: US1 (T024-T041) - **3 days**
4. Phase 4: US2 (T042-T049) - **1 day**
5. Select Phase 11 tasks: T131 (full stack test) - **0.5 days**

**Total MVP Timeline**: ~7.5 days (assuming single developer, sequential execution)

### Full Feature Timeline Estimate

- **Phase 1**: 1 day
- **Phase 2**: 2 days
- **Phase 3 (US1)**: 3 days
- **Phase 4 (US2)**: 1 day
- **Phase 5 (US3)**: 2 days
- **Phase 6 (US4)**: 2 days
- **Phase 7 (US5)**: 2 days
- **Phase 8 (US6)**: 1 day
- **Phase 9 (US7)**: 0.5 days
- **Phase 10 (US8)**: 1 day
- **Phase 11 (Polish)**: 2 days

**Total**: ~17.5 days (sequential) or ~10-12 days (with parallelization)

---

## Parallel Execution Examples

### Phase 2 Parallelization

Can run concurrently after T013 (MCP Server base):
```bash
# Terminal 1
Task T014: MCP Qdrant proxy
Task T018: Embedding service chunking

# Terminal 2
Task T015: MCP Neon proxy
Task T019: Embedding service embeddings

# Terminal 3
Task T016: MCP OpenAI proxy
Task T022: Database migration SQL
```

### Phase 3 (US1) Parallelization

After T024 (Conversation model):
```bash
# Backend Team
T025: DocumentChunk model
T026-T028: RAG service methods (sequential within team)

# API Team (after T026-T028)
T029-T032: Chat endpoints (can parallelize if different files)

# Frontend Team
T033: ChatMessage component
T034: CitationList component
T035-T038: ChatWithBook main component (sequential)
```

### Phase 5 (US3) + Phase 6 (US4) Parallel Development

**US3 (Auth) and US4 (Personalization) can start simultaneously**:
```bash
# Team A: Authentication
T050-T059: Backend auth (models, service, endpoints)
T060-T068: Frontend auth (modal, context, integration)

# Team B: Personalization (can start backend work)
T069-T074: Backend personalization (model, service, endpoints)
# Wait for Team A to complete T064 (AuthContext) before:
T075-T081: Frontend personalization (button, handlers)
```

---

## Implementation Strategy

### Recommended Approach: Incremental Delivery

1. **Week 1: MVP (US1 + US2)**
   - Setup + Foundational + Core chatbot + Selected text
   - Deploy to staging, gather feedback

2. **Week 2: Authentication & Personalization (US3 + US4)**
   - Add user accounts and background-based personalization
   - Deploy, test with beta users

3. **Week 3: Translation & Persistence (US5 + US6 + US8)**
   - Add Urdu translation and conversation history
   - Complete P1 feature set

4. **Week 4: Enhancements & Polish (US7 + Phase 11)**
   - Add dedicated page, write tests, optimize
   - Production deployment

### Testing Strategy

- **Unit Tests**: Write alongside implementation (T116-T119)
- **Integration Tests**: After each user story phase (T120-T122)
- **Contract Tests**: Before deployment (T123)
- **Manual E2E Tests**: Included in each user story phase (T041, T049, T068, etc.)
- **Performance Tests**: Validate success criteria in Phase 11 (T140)

### Risk Mitigation

1. **Better Auth Integration Risk** (T051-T054):
   - If Better Auth proves incompatible, fallback to custom JWT auth
   - Estimated 1-day pivot time

2. **OpenAI Rate Limits** (T016):
   - Implement exponential backoff immediately
   - Monitor usage, add queue if needed

3. **Urdu Translation Quality** (T083):
   - Manual review of first 3 translations
   - Iterate on prompts before scaling

4. **Free Tier Limits** (Qdrant, Neon):
   - Monitor usage from day 1
   - Implement per-user rate limiting (T127)

---

## Success Metrics Validation (Phase 11, Task T140)

Map to spec.md Success Criteria:

- **SC-001**: Response time <5s → Backend logging + frontend timing
- **SC-002**: 90% citation rate → Count citations in Message.citations
- **SC-003**: Widget load <2s → Frontend performance profiling
- **SC-004**: 100% embedding success → Pipeline script output
- **SC-005**: Works without Neon → Manual test with DATABASE_URL unset
- **SC-006**: Conversation persists 5+ navigations → E2E test
- **SC-007**: 30% relevance improvement with selection → A/B test
- **SC-008**: Cross-browser compatible → Manual test 4 browsers
- **SC-009**: 100% MCP usage → Code audit (no direct service calls)
- **SC-010**: Zero exposed API keys → Static analysis + review
- **SC-011**: Signup <2min → User timing test
- **SC-012**: 80% personalization satisfaction → Post-interaction survey
- **SC-013**: 100% code preservation → Regex validation automated
- **SC-014**: Personalization <10s → Backend timing logs
- **SC-015**: History accessible → Database query test
- **SC-016**: Anonymous chatbot works → Manual test logged out
- **SC-017**: Urdu semantic accuracy → Back-translation + manual review

---

**Tasks Generated**: 2025-12-10
**Ready for Implementation**: Yes
**Next Command**: Start with Phase 1 (T001-T012)
