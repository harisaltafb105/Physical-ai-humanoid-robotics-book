# Feature Specification: RAG Chatbot for Physical AI Robotics Book

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-10
**Updated**: 2025-12-10
**Status**: Draft
**Input**: User description: "Build and embed a Retrieval-Augmented Generation (RAG) chatbot within this published book. The chatbot must be implemented using OpenAI Agents or ChatKit SDKs, use FastAPI as the backend server, and rely on Qdrant Cloud Free Tier for embeddings and vector search. It must also integrate Neon Serverless Postgres for chat history and session tracking (the system must fully function even without SQL storage). All responses from the chatbot must rely strictly on the book's content, including text explicitly selected by the user. The project must also include an end-to-end chunking and embedding pipeline for all Markdown pages. A Docusaurus React component named ChatWithBook must be created for the UI, along with: A dedicated page titled AI Chatbot; A floating widget visible across all pages. **Mandatory Features:** Implement Signup and Signin using Better Auth (no keys required) with software & hardware background collection at signup; Allow logged-in users to personalize chapter content via a button at the start of each chapter; Allow logged-in users to translate chapter content into Urdu via a button at the start of each chapter. All external services (Qdrant, Neon, and model access) must be accessed via an MCP Server running at Context Level 7, ensuring unified project-wide integration."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Book Content (Priority: P1)

Readers of the Physical AI and Humanoid Robotics book want to quickly find answers to specific questions about robotics concepts, ROS2 commands, simulation setup, or VLA integration without manually searching through multiple chapters.

**Why this priority**: This is the core value proposition - enabling readers to get instant, contextual answers from the book's content. Without this, the chatbot provides no value.

**Independent Test**: Can be fully tested by asking various questions (e.g., "How do I set up ROS2?", "What is VSLAM?") and verifying responses contain relevant excerpts from the correct book chapters. Delivers immediate value by providing accurate answers.

**Acceptance Scenarios**:

1. **Given** a reader is viewing any page in the book, **When** they ask "What are the basic ROS2 concepts?", **Then** the chatbot retrieves relevant content from Module 1 and provides a coherent answer with source references
2. **Given** a reader asks about Isaac Sim setup, **When** the query is processed, **Then** the response includes step-by-step instructions from the Isaac module with accurate page/section citations
3. **Given** a reader asks an ambiguous question, **When** the chatbot cannot find sufficient context, **Then** it acknowledges limitations and suggests related topics from the book
4. **Given** a reader is on the URDF chapter, **When** they ask "Explain this section" without specific context, **Then** the chatbot prioritizes content from the current chapter in its response

---

### User Story 2 - Query with Selected Text Context (Priority: P1)

Readers want to highlight a specific paragraph or code snippet in the book and ask the chatbot to explain, expand, or provide related information based on that exact selection.

**Why this priority**: This dramatically improves answer accuracy by providing explicit context. It's a critical differentiator from generic chatbots and directly addresses the requirement for responses based on "text explicitly selected by the user."

**Independent Test**: Can be tested by selecting text from any chapter (e.g., a URDF code block), asking "Explain this code", and verifying the response references the selected text and provides relevant explanation from surrounding content.

**Acceptance Scenarios**:

1. **Given** a reader selects a code snippet about ROS2 publishers, **When** they click "Ask about this" and submit "What does this code do?", **Then** the chatbot's response directly references the selected code and explains it using concepts from the same chapter
2. **Given** a reader highlights a paragraph about sensor fusion, **When** they ask "How does this relate to navigation?", **Then** the response connects the selected text to navigation concepts from the relevant module
3. **Given** a reader selects text from multiple paragraphs, **When** the selection exceeds a reasonable token limit, **Then** the system truncates gracefully and notifies the user
4. **Given** a reader selects text and then navigates to a different page, **When** they ask a question, **Then** the system retains the selected context for the current session

---

### User Story 3 - Sign Up and Sign In with Better Auth (Priority: P1)

New readers want to create an account to access personalized features, while returning users want to sign in seamlessly to continue their learning journey with their preferences and history intact.

**Why this priority**: Authentication is a prerequisite for personalization, translation, and persistent chat history. It enables the system to tailor content to individual user backgrounds and maintain long-term engagement.

**Independent Test**: Can be tested by completing the signup flow with software/hardware background inputs, signing out, and signing back in to verify session persistence and profile data retention. Delivers value by enabling all personalized features.

**Acceptance Scenarios**:

1. **Given** a new reader visits the book site, **When** they click "Sign Up", **Then** they see a registration form requesting email, password, software background (beginner/intermediate/advanced), and hardware background (simulation-only/hobbyist/professional)
2. **Given** a reader completes the signup form with valid inputs, **When** they submit, **Then** their account is created without requiring external API keys, and they are automatically signed in
3. **Given** a returning reader clicks "Sign In", **When** they enter valid credentials, **Then** they are authenticated and their previous conversations and preferences are loaded
4. **Given** a reader is signed in, **When** they close the browser and return later, **Then** their session is restored if within the session timeout period
5. **Given** a reader attempts signup with an existing email, **When** they submit, **Then** they receive a clear error message indicating the account already exists

---

### User Story 4 - Personalize Chapter Content Based on Background (Priority: P1)

Logged-in readers with varying technical backgrounds want chapter content adapted to their skill level so they can learn at an appropriate pace without being overwhelmed or under-challenged.

**Why this priority**: Personalization is a mandatory feature that significantly improves learning outcomes by matching content complexity to user expertise. A beginner needs more explanations; a professional needs concise, advanced insights.

**Independent Test**: Can be tested by signing in with different background profiles (beginner vs. advanced), clicking the "Personalize" button at the start of a chapter, and verifying the content adjusts appropriately. Delivers value by making the book accessible to diverse audiences.

**Acceptance Scenarios**:

1. **Given** a logged-in reader with "beginner" software background is viewing a chapter, **When** they click the "Personalize" button at the chapter start, **Then** the content is regenerated with simplified explanations, additional context for technical terms, and step-by-step guidance
2. **Given** a logged-in reader with "advanced" software background personalizes a chapter, **When** the content is regenerated, **Then** it focuses on advanced concepts, best practices, and optimization techniques with minimal introductory material
3. **Given** a reader with "simulation-only" hardware background personalizes a chapter on physical robots, **When** the content loads, **Then** it emphasizes simulation environments and virtual testing over physical hardware setup
4. **Given** a reader personalizes a chapter, **When** they navigate away and return, **Then** the personalized version persists for their session unless they reset it
5. **Given** a reader who is not logged in clicks a "Personalize" button, **Then** they are prompted to sign in or sign up first

---

### User Story 5 - Translate Chapter Content to Urdu (Priority: P1)

Logged-in readers who are native Urdu speakers or prefer reading in Urdu want to translate chapter content into Urdu to better understand complex robotics concepts in their preferred language.

**Why this priority**: Translation to Urdu is a mandatory feature that expands accessibility to Urdu-speaking audiences, removing language barriers to learning advanced robotics topics.

**Independent Test**: Can be tested by signing in, clicking the "Translate to Urdu" button at the start of a chapter, and verifying the content is accurately translated while preserving code blocks, technical terms, and formatting. Delivers value by making the book accessible to non-English speakers.

**Acceptance Scenarios**:

1. **Given** a logged-in reader is viewing any chapter, **When** they click the "Translate to Urdu" button at the chapter start, **Then** all text content (headings, paragraphs, lists) is translated to Urdu while code blocks and technical identifiers remain in English
2. **Given** a reader has translated a chapter to Urdu, **When** they click "View Original" or "Back to English", **Then** the chapter reverts to the original English content immediately
3. **Given** a reader translates a chapter, **When** they use the chatbot to ask questions, **Then** the chatbot can accept queries in Urdu and respond in Urdu based on the translated content
4. **Given** a reader who is not logged in attempts to translate a chapter, **When** they click the translate button, **Then** they are prompted to sign in or sign up first
5. **Given** a translated chapter contains code snippets, **When** the translation is applied, **Then** code syntax, variable names, and function names remain unchanged in English to maintain technical accuracy

---

### User Story 6 - Persistent Chat via Floating Widget (Priority: P2)

Readers want to continue a conversation with the chatbot as they navigate between different chapters and pages without losing context or having to re-ask questions.

**Why this priority**: Enhances user experience by making the chatbot always accessible and maintaining conversation continuity. Essential for multi-step learning workflows (e.g., "First, explain URDF, then show me how to use it in Gazebo").

**Independent Test**: Can be tested by starting a conversation on one page, navigating to another page, and verifying the conversation history persists and the widget remains accessible. Delivers value by enabling multi-turn, context-aware assistance.

**Acceptance Scenarios**:

1. **Given** a reader opens the floating widget on the Quick Start page, **When** they ask two questions and then navigate to Module 2, **Then** the widget remains visible and the conversation history is preserved
2. **Given** a reader has an active conversation in the widget, **When** they close the widget and reopen it within the same session, **Then** the full conversation history is displayed
3. **Given** a reader is using the widget, **When** they click anywhere outside the widget, **Then** the widget minimizes but remains accessible via a fixed button/icon
4. **Given** a reader has multiple browser tabs open with different book pages, **When** they interact with the widget in one tab, **Then** each tab maintains independent conversation state

---

### User Story 7 - Access Dedicated Chatbot Page (Priority: P3)

Readers who prefer a full-screen chat interface want to access a dedicated "AI Chatbot" page where they can interact with the chatbot without the distraction of book content.

**Why this priority**: Provides an alternative interface for users who prefer focused chat sessions, but not critical for MVP since the floating widget covers most use cases.

**Independent Test**: Can be tested by navigating to the "AI Chatbot" page, verifying the chat interface is fully functional, and that conversations started here are independent of the floating widget sessions.

**Acceptance Scenarios**:

1. **Given** a reader navigates to the "AI Chatbot" page from the main menu, **When** the page loads, **Then** they see a clean chat interface with an introduction message about the chatbot's capabilities
2. **Given** a reader is on the dedicated chatbot page, **When** they ask questions, **Then** the experience is identical to the floating widget in terms of response quality and source citations
3. **Given** a reader uses both the dedicated page and floating widget, **When** they switch between them, **Then** conversations remain separate unless explicitly linked via session management

---

### User Story 8 - Review Conversation History Across Sessions (Priority: P3)

Returning readers want to review their previous conversations with the chatbot to revisit explanations or continue learning from where they left off.

**Why this priority**: Adds significant value for repeat users and supports learning continuity, but the system must function without this feature (as per "fully function even without SQL storage").

**Independent Test**: Can be tested by having a conversation, closing the browser, returning later, and verifying the conversation history is accessible (if session tracking is enabled).

**Acceptance Scenarios**:

1. **Given** session tracking is enabled and a reader returns to the site, **When** they open the chatbot widget, **Then** they see an option to "Load previous conversations"
2. **Given** a reader selects a previous conversation from history, **When** it loads, **Then** all messages and context are displayed accurately
3. **Given** session tracking is disabled (SQL storage unavailable), **When** a reader refreshes the page, **Then** the chatbot starts a fresh session without errors or degraded functionality
4. **Given** a reader has multiple conversation histories, **When** they view the list, **Then** each conversation is labeled with date/time and first query for easy identification

---

### Edge Cases

- What happens when the book content is updated with new chapters? Will the embedding pipeline automatically detect and process new Markdown files?
- How does the system handle malformed or extremely long user queries (e.g., 10,000+ character inputs)?
- What happens if Qdrant Cloud service is temporarily unavailable? Does the chatbot degrade gracefully or return an error?
- How are concurrent users handled? If 100 readers ask questions simultaneously, does the system maintain acceptable response times?
- What happens when a user selects text in a language other than English or includes special characters/code syntax that might break parsing?
- How does the system handle questions that require combining information from multiple distant chapters (e.g., "How do Isaac Sim and ROS2 work together?")?
- What happens if the MCP Server at Context Level 7 fails to connect to Qdrant, Neon, or the AI model provider?
- How does personalization handle users who select conflicting backgrounds (e.g., "beginner" software but "professional" hardware)?
- What happens when a user requests Urdu translation for a chapter with complex diagrams or embedded images with English text?
- How does the system handle users who sign up with the same email using different authentication providers (if supported in the future)?
- What happens if a user personalizes or translates a chapter and then that chapter's source content is updated?
- How does the chatbot maintain context when a user switches between English and Urdu translations mid-conversation?
- What happens if Better Auth service is unavailable? Can users still access non-authenticated features (basic chatbot)?
- How are incomplete signup forms handled (e.g., user skips software/hardware background selection)?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST process all Markdown files in the `docs/` directory and generate text chunks optimized for semantic search
- **FR-002**: System MUST create embeddings for each chunk using a model compatible with Qdrant Cloud and store them in a Qdrant vector database collection
- **FR-003**: System MUST accept user queries via a REST API endpoint and perform vector similarity search against the embedded book content
- **FR-004**: System MUST generate responses using an AI language model (OpenAI Agents or ChatKit SDK) that incorporates retrieved book chunks as context
- **FR-005**: System MUST provide source citations for each response, including chapter name, section, and relevance score
- **FR-006**: System MUST accept selected text from the user interface and prioritize it in the retrieval context
- **FR-007**: System MUST render a React component named "ChatWithBook" that provides a chat interface within the Docusaurus site
- **FR-008**: System MUST display a floating chat widget accessible from all pages with minimize/maximize functionality
- **FR-009**: System MUST provide a dedicated route/page titled "AI Chatbot" with a full-screen chat interface
- **FR-010**: System MUST maintain conversation state during browser navigation across different book pages
- **FR-011**: System MUST function fully even when Neon Postgres is unavailable, using in-memory or session-based storage as fallback
- **FR-012**: System MUST route all interactions with Qdrant Cloud, Neon Postgres, and AI model APIs through an MCP Server at Context Level 7
- **FR-013**: System MUST validate and sanitize all user inputs before processing to prevent injection attacks
- **FR-014**: System MUST handle API rate limits gracefully by implementing retry logic with exponential backoff
- **FR-015**: System MUST log all chatbot interactions (queries, responses, errors) for debugging and quality improvement
- **FR-016**: Chunking pipeline MUST preserve code blocks, headings, and other Markdown formatting elements as metadata for improved retrieval
- **FR-017**: System MUST detect when retrieved content is insufficient to answer a query and provide appropriate fallback responses
- **FR-018**: System MUST support pagination or streaming for long responses to avoid frontend timeout issues
- **FR-019**: System MUST provide signup functionality using Better Auth that collects email, password, software background (beginner/intermediate/advanced), and hardware background (simulation-only/hobbyist/professional)
- **FR-020**: System MUST provide signin functionality that authenticates users and restores their session, preferences, and conversation history
- **FR-021**: System MUST render "Personalize" and "Translate to Urdu" buttons at the start of each chapter, visible only to logged-in users
- **FR-022**: System MUST regenerate chapter content based on user's software and hardware background when the "Personalize" button is clicked
- **FR-023**: System MUST translate chapter content to Urdu when the "Translate to Urdu" button is clicked, preserving code blocks and technical terms in English
- **FR-024**: System MUST provide a toggle to revert translated content back to the original English version
- **FR-025**: System MUST store user profiles (background preferences) in Neon Postgres when available, with fallback to session storage
- **FR-026**: System MUST allow unauthenticated users to access basic chatbot functionality without personalization or translation features
- **FR-027**: System MUST validate user background selections during signup and provide default values if inputs are incomplete
- **FR-028**: Chatbot MUST support Urdu language queries and responses when a user is viewing Urdu-translated content
- **FR-029**: System MUST cache personalized and translated chapter versions per user session to avoid redundant regeneration
- **FR-030**: System MUST integrate Better Auth without requiring external API keys or third-party authentication providers

### Non-Functional Requirements

- **NFR-001**: Chatbot responses MUST be generated within 5 seconds for 95% of queries under normal load
- **NFR-002**: The embedding pipeline MUST process all existing book content (approximately 20-50 Markdown files) within 10 minutes
- **NFR-003**: The system MUST support at least 50 concurrent users without degradation in response time
- **NFR-004**: All API keys and credentials MUST be stored securely using environment variables, never hardcoded
- **NFR-005**: The Docusaurus build process MUST complete successfully with the ChatWithBook component integrated
- **NFR-006**: The floating widget MUST be responsive and functional on mobile, tablet, and desktop viewports
- **NFR-007**: The system MUST provide meaningful error messages to users when services are unavailable
- **NFR-008**: User signup and signin MUST complete within 3 seconds under normal conditions
- **NFR-009**: Chapter personalization MUST regenerate content within 10 seconds for 90% of requests
- **NFR-010**: Urdu translation MUST complete within 8 seconds for average-length chapters (2000-3000 words)
- **NFR-011**: Better Auth integration MUST function without requiring user-provided API keys or credentials
- **NFR-012**: Personalized and translated content MUST maintain readability and formatting consistency with the original

### Key Entities

- **DocumentChunk**: Represents a segment of book content with text, metadata (chapter, section, headings), embedding vector, and unique identifier
- **ChatMessage**: Represents a single message in a conversation with role (user/assistant), content, timestamp, and optional source citations
- **Conversation**: Represents a chat session with unique identifier, list of messages, creation timestamp, and optional user identifier
- **SelectedContext**: Represents text explicitly highlighted by the user with content, source page URL, and selection timestamp
- **EmbeddingMetadata**: Represents metadata for each chunk including chapter title, module number, heading hierarchy, and code block indicators
- **UserProfile**: Represents a registered user with unique identifier, email, software background (beginner/intermediate/advanced), hardware background (simulation-only/hobbyist/professional), and account creation timestamp
- **PersonalizedContent**: Represents a customized version of a chapter with original chapter identifier, user profile reference, regenerated content, and personalization timestamp
- **TranslatedContent**: Represents an Urdu translation of a chapter with original chapter identifier, translated text, code block preservation flags, and translation timestamp

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can receive relevant answers to robotics questions from the book within 5 seconds of submitting a query
- **SC-002**: At least 90% of chatbot responses include direct citations to specific book sections or chapters
- **SC-003**: The floating widget loads and becomes interactive within 2 seconds of page load
- **SC-004**: The embedding pipeline successfully processes and indexes 100% of Markdown files in the `docs/` directory without errors
- **SC-005**: The system remains fully functional (chat, retrieval, response generation) when Neon Postgres is disconnected, demonstrating graceful degradation
- **SC-006**: Users can maintain continuous conversations across at least 5 page navigations without losing context
- **SC-007**: Selected text context improves answer relevance by at least 30% compared to queries without selection (measured by citation accuracy)
- **SC-008**: The dedicated "AI Chatbot" page and floating widget are accessible and functional on Chrome, Firefox, Safari, and Edge browsers
- **SC-009**: The MCP Server successfully handles 100% of external service requests without bypassing the Context Level 7 integration layer
- **SC-010**: Zero API keys or credentials are exposed in client-side code or public repositories
- **SC-011**: Users can complete signup with background information in under 2 minutes
- **SC-012**: At least 80% of users who personalize a chapter report that the content matches their skill level appropriately
- **SC-013**: Urdu translations preserve 100% of code blocks and technical identifiers in their original English form
- **SC-014**: Personalized content generates within 10 seconds for 90% of chapter personalization requests
- **SC-015**: Logged-in users can access their conversation history from previous sessions when Neon Postgres is available
- **SC-016**: The system functions fully for unauthenticated users (chatbot queries work) even when Better Auth or Neon Postgres are unavailable
- **SC-017**: Chatbot responses in Urdu maintain semantic accuracy compared to English responses for the same query

### Assumptions

- The book content is authored in standard Markdown format with consistent heading structures
- Qdrant Cloud Free Tier provides sufficient storage and query capacity for the book's content volume (estimated 50-100 embedded chunks)
- Users have modern browsers with JavaScript enabled
- The Docusaurus site is already deployed and operational; this feature enhances it rather than creating a new site
- The MCP Server infrastructure at Context Level 7 is already configured or will be set up as part of this feature
- OpenAI API or ChatKit SDK access is available with appropriate rate limits for the expected user base
- Readers primarily ask questions in English or Urdu
- The FastAPI backend will be deployed on infrastructure capable of handling concurrent requests (e.g., cloud hosting with auto-scaling)
- Better Auth is available and functional without requiring external authentication provider API keys
- AI models used for personalization and translation can generate content that matches user backgrounds and accurately translate English to Urdu
- Users provide honest and accurate information about their software and hardware backgrounds during signup
- Urdu-speaking users are comfortable with code blocks and technical terms remaining in English within translated content

### Out of Scope

- Multi-language support beyond English and Urdu (e.g., Arabic, Hindi, Spanish)
- Voice input/output for the chatbot
- Integration with external knowledge bases beyond the book's content
- Social authentication providers (Google, GitHub, Facebook) - Better Auth is used without external providers
- Analytics dashboard for tracking chatbot usage patterns, personalization effectiveness, or translation quality
- Automated testing of chatbot response quality against ground truth Q&A pairs
- Real-time collaborative chat between multiple users
- Export of conversations to PDF or other formats
- User-editable profiles (changing background selections after signup)
- Translation to languages other than Urdu
- Personalization based on learning progress or quiz results (only background-based personalization)
- Content recommendations based on user history or preferences
