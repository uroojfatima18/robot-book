# Feature Specification: Integrated RAG Chatbot with Authentication System

**Feature Branch**: `006-rag-chatbot-auth`
**Created**: 2026-01-02
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot + Signup System for robotics book UI with OpenAI Agents, Qdrant Cloud vector storage, Neon PostgreSQL database, and secure JWT-based authentication"

## Overview

A Retrieval-Augmented Generation (RAG) chatbot integrated into the robotics book web interface that answers questions exclusively from book content. The system includes secure user authentication with email verification, role-based access control, and intelligent guardrails to prevent responses outside the provided material.

## Clarifications

### Session 2026-01-02

- Q: Which embedding model should the RAG system use? -> A: text-embedding-3-small (1536 dimensions)
- Q: What JWT token lifetimes should the system use? -> A: Access token 15 minutes, Refresh token 7 days
- Q: Which authentication library should handle signup/login? -> A: Better Auth library
- Q: Which email service for verification emails? -> A: Resend
- Q: What cosine similarity threshold triggers "No matching answer"? -> A: 0.7

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Reader Asks Question from Book Content (Priority: P1)

A verified book reader wants to ask a question about robotics concepts and receive an accurate answer sourced exclusively from the book content.

**Why this priority**: This is the core value proposition - users purchase/access the book to learn, and the chatbot provides instant, contextual answers that enhance their learning experience.

**Independent Test**: Can be fully tested by asking a question about ROS 2 concepts (covered in book) and verifying the answer cites relevant book sections.

**Acceptance Scenarios**:

1. **Given** a verified logged-in user viewing Chapter 3, **When** they select a paragraph about sensor fusion and click "Ask about this", **Then** the chatbot displays a contextual answer derived only from the selected text and related book content within 5 seconds.
2. **Given** a verified logged-in user, **When** they type a question about navigation algorithms, **Then** the chatbot returns a structured answer with bullet points/steps sourced from book content, or clearly states "No matching answer found in book" if the topic is not covered.
3. **Given** a verified logged-in user, **When** they ask about an unrelated topic (e.g., "What is the weather?"), **Then** the chatbot responds "This question is outside the provided material."

---

### User Story 2 - New User Registration and Verification (Priority: P1)

A new visitor wants to create an account to access the chatbot feature, completing email verification before gaining chatbot access.

**Why this priority**: Authentication is required for chatbot access - without it, users cannot use the primary feature. Critical for security and user management.

**Independent Test**: Can be fully tested by completing signup flow, receiving verification email, clicking link, and confirming account status changes to verified.

**Acceptance Scenarios**:

1. **Given** a visitor on the book website, **When** they complete the signup form with valid email and password, **Then** the system creates an unverified account and sends a verification email within 30 seconds.
2. **Given** a user with an unverified account, **When** they attempt to access the chatbot, **Then** they see the message "Please verify your email to use the chatbot."
3. **Given** a user who received a verification email, **When** they click the verification link, **Then** their account status changes to verified and they can access the chatbot.

---

### User Story 3 - User Login and Session Management (Priority: P2)

A registered user wants to log in securely and maintain their session across browser sessions without frequent re-authentication.

**Why this priority**: Essential for user experience - users should not need to re-login frequently, but security must be maintained.

**Independent Test**: Can be fully tested by logging in, closing browser, returning, and verifying session persists via refresh token mechanism.

**Acceptance Scenarios**:

1. **Given** a verified user, **When** they enter valid credentials, **Then** they are logged in and receive access to the chatbot within 2 seconds.
2. **Given** a logged-in user whose access token expires, **When** they make a request, **Then** the system automatically refreshes their session without requiring re-login.
3. **Given** a user with invalid credentials, **When** they attempt to log in, **Then** they receive a generic error message "Invalid email or password" (not revealing which field is incorrect).

---

### User Story 4 - Admin Ingests Book Content (Priority: P2)

An administrator wants to add or update book content in the chatbot knowledge base so readers can ask questions about new chapters.

**Why this priority**: Without content ingestion, the chatbot has no knowledge to draw from. This enables the core RAG functionality.

**Independent Test**: Can be fully tested by ingesting a chapter text and then verifying questions about that chapter return relevant answers.

**Acceptance Scenarios**:

1. **Given** an admin user, **When** they submit chapter text through the ingest interface, **Then** the content is processed, chunked, and stored with metadata (chapter, source, location) within 60 seconds.
2. **Given** an admin who ingested new content, **When** a reader asks about topics in that content, **Then** the chatbot retrieves and uses the newly ingested content in responses.
3. **Given** a non-admin user, **When** they attempt to access the ingest functionality, **Then** they receive an authorization error.

---

### User Story 5 - Conversation Logging for User History (Priority: P3)

A user wants to see their previous questions and answers to review their learning progress.

**Why this priority**: Enhances user experience and provides value for returning users, but not critical for core chatbot functionality.

**Independent Test**: Can be fully tested by asking multiple questions, navigating to history, and verifying all Q&A pairs are displayed.

**Acceptance Scenarios**:

1. **Given** a logged-in user who has asked questions, **When** they view their conversation history, **Then** they see a chronological list of their questions and the chatbot answers.
2. **Given** a user viewing their history, **When** they select a previous conversation, **Then** they can view the full context of that exchange.

---

### Edge Cases

- What happens when a user selects text spanning multiple chapters? System should combine context from both chapters for the response.
- How does the system handle when no relevant results are found (similarity below 0.7 threshold)? Returns "No matching answer found in book."
- What happens if a user exceeds the rate limit (10 messages/minute)? Returns a friendly message indicating they should wait, with seconds remaining.
- What happens if context exceeds 2,000 tokens? System truncates to most relevant chunks while maintaining coherence.
- How does the system handle ambiguous questions? Chatbot asks clarifying questions before attempting to answer.
- What happens if the user verification link expires? User can request a new verification email.
- How does the system handle concurrent login attempts from different devices? Both sessions are valid; refresh tokens are device-specific.

## Requirements *(mandatory)*

### Functional Requirements

**Authentication and Authorization**

- **FR-001**: System MUST use Better Auth library for user authentication and session management.
- **FR-002**: System MUST allow users to create accounts using email and password only (no social login initially).
- **FR-003**: System MUST hash passwords securely using Better Auth built-in secure hashing.
- **FR-004**: System MUST send verification emails via Resend to new accounts within 30 seconds of registration.
- **FR-005**: System MUST block unverified accounts from accessing chatbot functionality.
- **FR-006**: System MUST issue access tokens (15-minute lifetime) and refresh tokens (7-day lifetime) upon successful login.
- **FR-007**: System MUST support two roles: user (read-only, ask questions) and admin (manage content, approve embeddings).
- **FR-008**: System MUST invalidate sessions when users explicitly log out.
- **FR-009**: System MUST provide password reset functionality via email verification.

**RAG Pipeline**

- **FR-010**: System MUST accept user-selected text from the book interface and use it as primary context for answers.
- **FR-011**: System MUST clean and chunk ingested text into segments of 400-700 tokens.
- **FR-012**: System MUST generate embeddings using OpenAI text-embedding-3-small (1536 dimensions) for text chunks and store them with metadata (chapter, source, location, page_id).
- **FR-013**: System MUST retrieve top 5 most relevant chunks using cosine similarity search when answering questions.
- **FR-014**: System MUST respond "No matching answer found in book" when no relevant context exists (similarity score below 0.7 threshold).
- **FR-015**: System MUST only answer from retrieved context - no hallucination or external knowledge allowed.

**Chatbot Behavior**

- **FR-016**: System MUST refuse to answer questions outside book content with: "This question is outside the provided material."
- **FR-017**: System MUST detect ambiguous questions and ask for clarification before answering.
- **FR-018**: System MUST format responses appropriately (bullet points, step-by-step breakdowns, code examples when relevant).
- **FR-019**: System MUST stream responses token-by-token to the user interface.
- **FR-020**: System MUST enforce a rate limit of 10 messages per minute per user.
- **FR-021**: System MUST truncate context to 2,000 tokens maximum, prioritizing most relevant chunks.

**Logging and Audit**

- **FR-022**: System MUST log all user questions and chatbot answers with timestamps for registered users.
- **FR-023**: System MUST allow users to view their conversation history.

**Admin Functions**

- **FR-024**: Admin users MUST be able to ingest/update book content into the knowledge base.
- **FR-025**: Admin users MUST be able to view usage statistics and logs.

### Key Entities

- **User**: Represents a registered account with email, hashed password, role (user/admin), verification status, and creation timestamp. Managed by Better Auth.
- **Conversation Log**: Represents a Q&A exchange with user reference, question text, answer text, and timestamp.
- **Content Chunk**: Represents a segment of book content with metadata (chapter, source, location, page_id) and 1536-dimension embedding vector stored for retrieval.
- **Session**: Represents an authenticated session with access token (15-min expiry), refresh token (7-day expiry), and device information. Managed by Better Auth.

## Technical Stack & Integration

- **Authentication**: Better Auth library with email/password authentication
- **Email Service**: Resend for verification and password reset emails
- **Embedding Model**: OpenAI text-embedding-3-small (1536 dimensions)
- **Vector Database**: Qdrant Cloud for similarity search (cosine, threshold 0.7)
- **Primary Database**: Neon PostgreSQL for user accounts, sessions, and conversation logs
- **LLM Provider**: OpenAI Agents SDK for response generation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete account registration and email verification in under 3 minutes.
- **SC-002**: Chatbot responds to questions within 5 seconds for 95% of queries.
- **SC-003**: 95% of questions about book content return relevant, accurate answers (as measured by user feedback or manual review).
- **SC-004**: System supports at least 100 concurrent users without performance degradation.
- **SC-005**: 100% of questions outside book scope are correctly identified and refused.
- **SC-006**: Login process completes in under 2 seconds.
- **SC-007**: Automatic session refresh succeeds 99.9% of the time without user intervention.
- **SC-008**: Content ingestion completes within 60 seconds per chapter (average ~5,000 words).
- **SC-009**: Rate limiting correctly blocks users who exceed 10 messages per minute.
- **SC-010**: User conversation history loads within 2 seconds.

## Assumptions

- Users have valid email addresses and can receive verification emails.
- Book content is primarily text-based (code blocks, paragraphs) without complex multimedia requiring special embedding.
- The robotics book content is available in a processable text format for ingestion.
- Users access the system through modern web browsers with JavaScript enabled.
- Streaming responses are supported by the frontend framework.
- Resend email service is available and reliable for sending verification and password reset emails.
- Vector search with cosine similarity (threshold 0.7) and top-k=5 provides sufficient context for accurate answers.

## Out of Scope

- Social login (OAuth with Google, GitHub, etc.) - may be added in future iteration.
- Mobile native applications - web-responsive only.
- Multi-language support - English only for initial release.
- Voice input/output for chatbot interaction.
- Real-time collaborative features.
- Payment/subscription management for premium content.
- Advanced analytics dashboard for users.
- Bulk user import/export functionality.
