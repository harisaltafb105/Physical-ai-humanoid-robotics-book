# Data Model: RAG Chatbot with Auth, Personalization & Translation

**Feature**: 001-rag-chatbot
**Date**: 2025-12-10
**Source**: Derived from spec.md Key Entities and research.md decisions

## Entity Definitions

### 1. User

Represents a registered reader with authentication credentials and background profile.

**Fields**:
- `id`: UUID, primary key, auto-generated
- `email`: String (255), unique, not null, validated email format
- `password_hash`: String (255), not null, bcrypt hashed
- `software_background`: Enum ['beginner', 'intermediate', 'advanced'], not null
- `hardware_background`: Enum ['simulation-only', 'hobbyist', 'professional'], not null
- `created_at`: Timestamp, default now(), immutable
- `updated_at`: Timestamp, default now(), auto-update on modification

**Validation Rules**:
- Email must match regex: `^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$`
- Password minimum 8 characters before hashing
- Background enums strictly validated (no custom values)

**Relationships**:
- One-to-many with `Conversation` (user can have multiple conversations)
- One-to-many with `PersonalizedContent` (user can personalize multiple chapters)
- One-to-many with `TranslatedContent` (user can translate multiple chapters)

**State Transitions**:
- Created → Active (on successful signup)
- Active → Deleted (soft delete, retain conversation data for 30 days)

**Storage**:
- Primary: Neon Postgres `users` table
- Fallback: In-memory session storage (loses data on server restart)

---

### 2. Conversation

Represents a chat session between a user (or anonymous session) and the chatbot.

**Fields**:
- `id`: UUID, primary key, auto-generated
- `user_id`: UUID, foreign key to `User.id`, nullable (supports anonymous chat)
- `title`: String (255), nullable, auto-generated from first user query
- `language`: Enum ['en', 'ur'], default 'en', indicates conversation language
- `created_at`: Timestamp, default now(), immutable
- `updated_at`: Timestamp, default now(), auto-update on new message

**Validation Rules**:
- Title auto-generated as truncated first message (max 50 chars) if null
- Language must match ['en', 'ur']

**Relationships**:
- Many-to-one with `User` (optional, null for anonymous)
- One-to-many with `Message` (conversation contains messages)

**State Transitions**:
- Active → Archived (after 30 days of inactivity)
- Archived → Deleted (user-initiated or after 90 days)

**Storage**:
- Primary: Neon Postgres `conversations` table
- Fallback: In-memory map `{session_id: Conversation}`

---

### 3. Message

Represents a single message in a conversation (user query or assistant response).

**Fields**:
- `id`: UUID, primary key, auto-generated
- `conversation_id`: UUID, foreign key to `Conversation.id`, not null, cascade delete
- `role`: Enum ['user', 'assistant'], not null
- `content`: Text, not null, unlimited length
- `citations`: JSONB, nullable, array of source references
- `created_at`: Timestamp, default now(), immutable

**Validation Rules**:
- Content max length: 10,000 characters for user messages (prevent abuse)
- Content unlimited for assistant messages (responses can be long)
- Citations structure: `[{chapter_id, section, score}]`

**Relationships**:
- Many-to-one with `Conversation` (messages belong to conversation)

**Citations Schema**:
```json
{
  "citations": [
    {
      "chapter_id": "module-1-ros2/01-ros2-basics-first-node",
      "chapter_title": "ROS2 Basics: Your First Node",
      "section": "## Creating a Publisher",
      "relevance_score": 0.92,
      "excerpt": "To create a publisher in ROS2..."
    }
  ]
}
```

**Storage**:
- Primary: Neon Postgres `messages` table
- Fallback: In-memory array within `Conversation` object

---

### 4. DocumentChunk

Represents an embedded chunk of book content for vector search.

**Fields**:
- `id`: String, unique identifier (format: `{chapter_id}_{chunk_index}`)
- `chapter_id`: String, reference to source markdown file
- `module_name`: String, e.g., "Module 1: ROS2"
- `chapter_title`: String, e.g., "ROS2 Basics: Your First Node"
- `heading_hierarchy`: Array[String], e.g., ["## Creating a Publisher", "### Topic Configuration"]
- `content`: Text, 800 tokens average, 200 token overlap
- `has_code`: Boolean, true if chunk contains code blocks
- `code_language`: String, nullable, e.g., "python", "cpp"
- `embedding`: Vector(1536), float32 array from text-embedding-3-small
- `metadata`: JSONB, additional fields (line numbers, custom tags)
- `created_at`: Timestamp, chunk creation time
- `updated_at`: Timestamp, last re-embedding time

**Validation Rules**:
- Content length 400-1200 tokens (strict bounds)
- Embedding dimension exactly 1536
- Chapter_id must match existing docs/ file path

**Relationships**:
- No direct relationships (denormalized for performance)
- Linked to chapters via `chapter_id` string reference

**Metadata Schema**:
```json
{
  "line_start": 45,
  "line_end": 78,
  "tags": ["setup", "installation"],
  "difficulty": "beginner"
}
```

**Storage**:
- Primary: Qdrant Cloud collection `book_content`
- No fallback (vector search unavailable if Qdrant down)

---

### 5. PersonalizedContent

Represents a user-specific regenerated version of a chapter.

**Fields**:
- `id`: UUID, primary key, auto-generated
- `user_id`: UUID, foreign key to `User.id`, not null
- `chapter_id`: String, reference to original chapter
- `software_background`: Enum (cached from user profile at time of generation)
- `hardware_background`: Enum (cached from user profile at time of generation)
- `personalized_content`: Text, regenerated markdown content
- `created_at`: Timestamp, generation time
- `expires_at`: Timestamp, TTL for cache (created_at + 1 hour)

**Validation Rules**:
- Personalized content max size: 50KB (prevent excessive storage)
- Expires_at enforced by background cleanup job

**Relationships**:
- Many-to-one with `User` (user can personalize multiple chapters)

**Caching Strategy**:
- **In-Session Cache**: Redis or in-memory (preferred for free tier)
- **Persistent Cache**: Neon Postgres (optional, for longer TTL)
- **Key Format**: `personalized_{user_id}_{chapter_id}`
- **Eviction**: LRU or TTL-based (1 hour)

**Storage**:
- Primary: In-memory LRU cache (max 100 entries)
- Secondary: Neon Postgres `personalized_content` table (if available)
- Fallback: Regenerate on every request (no caching)

---

### 6. TranslatedContent

Represents an Urdu translation of a chapter.

**Fields**:
- `id`: UUID, primary key, auto-generated
- `user_id`: UUID, foreign key to `User.id`, nullable (can be shared across users)
- `chapter_id`: String, reference to original chapter
- `language`: Enum ['ur'], currently only Urdu
- `translated_content`: Text, Urdu markdown content
- `code_blocks_preserved`: Boolean, validation flag (must be true)
- `created_at`: Timestamp, translation time
- `expires_at`: Timestamp, TTL for cache (created_at + 1 hour)

**Validation Rules**:
- Translated content max size: 100KB (Urdu text uses more bytes)
- Code blocks preserved checked by regex: ` ```.*``` ` unchanged from original

**Relationships**:
- Many-to-one with `User` (optional, translations can be shared)

**Caching Strategy**:
- Same as PersonalizedContent
- **Key Format**: `translated_ur_{chapter_id}` (user-agnostic for reuse)
- **Shared Cache**: Same translation can serve multiple users

**Storage**:
- Primary: In-memory LRU cache (max 50 entries, larger than personalized due to sharing)
- Secondary: Neon Postgres `translated_content` table (if available)
- Fallback: Regenerate on every request

---

### 7. SelectedContext

Represents text explicitly highlighted by a user for contextual queries.

**Fields**:
- `session_id`: String, identifies browser session (UUID or cookie-based)
- `selected_text`: Text, max 2000 characters
- `source_page_url`: String, URL of page where text was selected
- `chapter_id`: String, derived from URL
- `timestamp`: Timestamp, selection time
- `expires_at`: Timestamp, selection context valid for 10 minutes

**Validation Rules**:
- Selected text max 2000 chars (prevent token overflow in LLM context)
- Timestamp enforced for expiration (old selections ignored)

**Relationships**:
- No persistent relationships (ephemeral, session-scoped)

**Storage**:
- Primary: In-memory session storage (browser localStorage + backend session)
- No persistent storage (cleared on page navigation after timeout)

---

### 8. EmbeddingMetadata

Represents metadata for each DocumentChunk to enhance retrieval accuracy.

**Fields**:
- `chunk_id`: String, foreign key to DocumentChunk.id
- `chapter_title`: String, denormalized for quick access
- `module_number`: Integer, e.g., 1, 2, 3
- `heading_hierarchy`: Array[String], denormalized
- `code_block_count`: Integer, number of code blocks in chunk
- `avg_sentence_length`: Float, readability metric
- `technical_terms`: Array[String], extracted keywords (e.g., ["URDF", "ROS2", "Gazebo"])

**Validation Rules**:
- Technical terms max 20 per chunk
- Module number 1-4 (based on book structure)

**Relationships**:
- One-to-one with `DocumentChunk` (metadata embedded in Qdrant payload)

**Storage**:
- Embedded in Qdrant Cloud as chunk payload (no separate table)

---

## Relationships Diagram

```
User (1) ----< (N) Conversation
User (1) ----< (N) PersonalizedContent
User (1) ----< (N) TranslatedContent

Conversation (1) ----< (N) Message

DocumentChunk (no relationships, standalone)
SelectedContext (no relationships, ephemeral)
EmbeddingMetadata (embedded in DocumentChunk)
```

---

## Database Schema (Neon Postgres)

```sql
-- Users table
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  software_background VARCHAR(50) NOT NULL CHECK (software_background IN ('beginner', 'intermediate', 'advanced')),
  hardware_background VARCHAR(50) NOT NULL CHECK (hardware_background IN ('simulation-only', 'hobbyist', 'professional')),
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- Conversations table
CREATE TABLE conversations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  title VARCHAR(255),
  language VARCHAR(2) DEFAULT 'en' CHECK (language IN ('en', 'ur')),
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- Messages table
CREATE TABLE messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
  role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
  content TEXT NOT NULL,
  citations JSONB,
  created_at TIMESTAMP DEFAULT NOW()
);

-- Personalized content cache (optional, for persistent caching)
CREATE TABLE personalized_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  chapter_id VARCHAR(255) NOT NULL,
  software_background VARCHAR(50),
  hardware_background VARCHAR(50),
  personalized_content TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  expires_at TIMESTAMP NOT NULL,
  UNIQUE(user_id, chapter_id)
);

-- Translated content cache (optional, for persistent caching)
CREATE TABLE translated_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chapter_id VARCHAR(255) NOT NULL UNIQUE,
  language VARCHAR(2) DEFAULT 'ur',
  translated_content TEXT NOT NULL,
  code_blocks_preserved BOOLEAN DEFAULT TRUE,
  created_at TIMESTAMP DEFAULT NOW(),
  expires_at TIMESTAMP NOT NULL
);

-- Indexes for performance
CREATE INDEX idx_conversations_user ON conversations(user_id, updated_at DESC);
CREATE INDEX idx_messages_conversation ON messages(conversation_id, created_at);
CREATE INDEX idx_personalized_user_chapter ON personalized_content(user_id, chapter_id);
CREATE INDEX idx_translated_chapter ON translated_content(chapter_id);
```

---

## Qdrant Collection Schema

```python
# Collection: book_content
{
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "payload_schema": {
    "chapter_id": "keyword",
    "module_name": "keyword",
    "chapter_title": "text",
    "heading_hierarchy": "keyword[]",
    "content": "text",
    "has_code": "bool",
    "code_language": "keyword",
    "metadata": {
      "line_start": "integer",
      "line_end": "integer",
      "tags": "keyword[]"
    }
  }
}
```

---

## Data Flow Examples

### Example 1: User Signup with Background

1. User submits signup form: `{email, password, software_bg: 'beginner', hardware_bg: 'hobbyist'}`
2. Backend validates inputs, hashes password
3. Insert into `users` table: `{id: UUID, email, password_hash, software_bg: 'beginner', hardware_bg: 'hobbyist'}`
4. Return session token (HTTP-only cookie)

### Example 2: Chatbot Query with Context

1. User selects text: "URDF stands for Unified Robot Description Format"
2. Frontend stores in `SelectedContext`: `{session_id, selected_text, source_page_url, timestamp}`
3. User asks: "How do I create a URDF file?"
4. Backend retrieves `SelectedContext` + performs vector search in Qdrant
5. Qdrant returns top 3 chunks: `[{chapter_id, content, score}]`
6. Backend constructs OpenAI prompt: `Selected context: {selected_text}\nBook content: {chunks}\nUser query: {query}`
7. OpenAI returns response with citations
8. Insert into `messages` table: `{conversation_id, role: 'assistant', content, citations}`

### Example 3: Chapter Personalization

1. Logged-in user (beginner/hobbyist) clicks "Personalize" on ROS2 chapter
2. Backend checks cache: `personalized_{user_id}_{chapter_id}`
3. Cache miss → fetch original chapter content
4. Construct system prompt: "You are adapting this robotics chapter for a beginner-level software engineer with hobbyist hardware experience..."
5. OpenAI regenerates chapter content
6. Store in cache: `{user_id, chapter_id, personalized_content, expires_at: now() + 1 hour}`
7. Return personalized markdown to frontend

### Example 4: Urdu Translation

1. Logged-in user clicks "Translate to Urdu" on Gazebo chapter
2. Backend checks shared cache: `translated_ur_{chapter_id}`
3. Cache miss → fetch original chapter content
4. Construct translation prompt: "Translate this chapter to Urdu, preserve code blocks..."
5. OpenAI returns Urdu translation
6. Validate code blocks unchanged (regex check)
7. Store in cache: `{chapter_id, language: 'ur', translated_content, expires_at: now() + 1 hour}`
8. Return Urdu markdown to frontend

---

## Storage Estimates (Free Tier Limits)

| Data Type | Storage per Item | Estimated Count | Total Storage | Storage Location |
|-----------|------------------|-----------------|---------------|------------------|
| User | 500 bytes | 1,000 users | 0.5 MB | Neon Postgres (0.5GB free) |
| Conversation | 200 bytes | 5,000 conversations | 1 MB | Neon Postgres |
| Message | 1 KB | 50,000 messages | 50 MB | Neon Postgres |
| DocumentChunk (embedding) | 6 KB | 250 chunks | 1.5 MB | Qdrant (1GB free) |
| PersonalizedContent (cached) | 30 KB | 100 entries | 3 MB | In-memory |
| TranslatedContent (cached) | 50 KB | 50 entries | 2.5 MB | In-memory |
| **Total Persistent Storage** | | | **52.5 MB** | Well within free tier |

---

## Validation and Constraints Summary

1. **User Email**: Unique, valid email format, max 255 chars
2. **Passwords**: Min 8 chars before hashing, bcrypt with salt
3. **Background Enums**: Strict validation, no custom values
4. **Message Content**: Max 10,000 chars for user, unlimited for assistant
5. **Selected Text**: Max 2000 chars, 10-minute expiration
6. **Chunk Content**: 400-1200 tokens, exactly 1536 dimensions
7. **Personalized Content**: Max 50KB, 1-hour TTL
8. **Translated Content**: Max 100KB, code blocks must be preserved
9. **Conversation Language**: Only 'en' or 'ur'
10. **Citations**: Valid JSONB array with required fields

---

## Next Steps

1. Define API contracts for FastAPI endpoints (contracts/)
2. Create quickstart guide for local development (quickstart.md)
3. Update agent context with data model schema
