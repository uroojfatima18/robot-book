# Tasks: RAG Chatbot with Authentication

**Input**: Design documents from `/specs/006-rag-chatbot-auth/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Key Focus**:
- **Vector Database (Qdrant Cloud)**: Store ENTIRE book content as embeddings
- **Book-Only Responses**: Chatbot MUST answer exclusively from ingested book content
- **Similarity Threshold**: 0.7 - reject queries with no matching content

**Tests**: NOT requested - implementation tasks only

**Organization**: Tasks grouped by user story for independent implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1-US5)
- Paths follow Next.js App Router structure per plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and core configuration

- [ ] T001 Create Next.js 14 project with App Router structure in src/
- [ ] T002 Install core dependencies: better-auth, drizzle-orm, @qdrant/js-client-rest, @google/genai, openai, resend
- [ ] T003 [P] Configure TypeScript strict mode in tsconfig.json
- [ ] T004 [P] Configure ESLint and Prettier in .eslintrc.js and .prettierrc
- [ ] T005 Create environment configuration with validation in src/lib/env.ts
- [ ] T006 [P] Copy TypeScript types from specs/006-rag-chatbot-auth/contracts/types.ts to src/types/index.ts
- [ ] T007 Create .env.example with all required environment variables (DATABASE_URL, QDRANT_URL, OPENAI_API_KEY, GEMINI_API_KEY, RESEND_API_KEY)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story

**CRITICAL**: No user story work can begin until this phase is complete

### Database Setup (Neon PostgreSQL + Drizzle)

- [ ] T008 Create Drizzle client with Neon serverless adapter in src/lib/db/index.ts
- [ ] T009 Create database schema with users, sessions, conversations, messages, rate_limits tables in src/lib/db/schema.ts
- [ ] T010 Create rate limit PostgreSQL function check_rate_limit() in src/lib/db/migrations/
- [ ] T011 Generate and run Drizzle migrations via pnpm db:push

### Vector Database Setup (Qdrant Cloud - Book Content Storage)

- [ ] T012 Create Qdrant client with API configuration in src/lib/qdrant.ts
- [ ] T013 Create Qdrant collection setup script for book_chunks (1536 dims, Cosine) in scripts/setup-qdrant.ts
- [ ] T014 Add pnpm script qdrant:setup to package.json

### Authentication Setup (Better Auth)

- [ ] T015 Create Better Auth configuration with email/password, JWT plugin in src/lib/auth.ts
- [ ] T016 Create Better Auth client-side helpers in src/lib/auth-client.ts
- [ ] T017 [P] Configure Resend email integration for verification emails in src/lib/auth.ts
- [ ] T018 Create auth API route handler in src/app/api/auth/[...all]/route.ts
- [ ] T019 Create auth middleware for protected routes in src/middleware.ts

### RAG Pipeline Core (Book-Only Responses)

- [ ] T020 Create OpenAI embeddings service (text-embedding-3-small, 1536 dims) in src/lib/embeddings.ts
- [ ] T021 Create text chunking utility (400-700 tokens with overlap) in src/lib/rag/chunk.ts
- [ ] T022 Create context truncation utility (max 2000 tokens, priority by score) in src/lib/rag/truncate.ts
- [ ] T023 Create vector retrieval service with 0.7 similarity threshold in src/lib/rag/retrieve.ts
- [ ] T024 Create Gemini streaming client with book-only system prompt in src/lib/gemini.ts
- [ ] T025 Create rate limiting utility with token bucket algorithm in src/lib/rate-limit.ts

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 4 - Admin Ingests Book Content (Priority: P2 - Moved First)

**Goal**: Enable admin to ingest ENTIRE book content into vector database so chatbot can answer from it

**Independent Test**: Ingest a chapter, then verify search returns relevant chunks

**Why First**: Without ingested book content, the chatbot has nothing to answer from. This MUST be done before US1.

### Implementation for User Story 4

- [ ] T026 [US4] Create admin authorization check utility in src/lib/auth-utils.ts
- [ ] T027 [US4] Implement content ingestion pipeline (chunk → embed → store) in src/lib/rag/ingest.ts
- [ ] T028 [US4] Create POST /api/admin/ingest endpoint in src/app/api/admin/ingest/route.ts
- [ ] T029 [US4] Create GET /api/admin/stats endpoint in src/app/api/admin/stats/route.ts
- [ ] T030 [US4] Create book content ingestion script for all chapters in scripts/ingest-book.ts
- [ ] T031 [US4] Add pnpm script ingest:book to ingest chapters from chapters/ directory
- [ ] T032 [US4] Ingest all existing book chapters (01-ros2-nervous-system, 02-digital-twin, 003-ai-robot-brain) into Qdrant

**Checkpoint**: Book content ingested into Qdrant - chatbot can now answer questions from book

---

## Phase 4: User Story 2 - New User Registration and Verification (Priority: P1)

**Goal**: Allow new visitors to create accounts with email verification before chatbot access

**Independent Test**: Complete signup, receive verification email, click link, confirm account verified

### Implementation for User Story 2

- [ ] T033 [P] [US2] Create AuthProvider context component in src/components/auth/AuthProvider.tsx
- [ ] T034 [P] [US2] Create SignupForm component with email/password fields in src/components/auth/SignupForm.tsx
- [ ] T035 [P] [US2] Create email verification message component in src/components/auth/VerifyEmailMessage.tsx
- [ ] T036 [US2] Create signup page in src/app/(auth)/signup/page.tsx
- [ ] T037 [US2] Create email verification page in src/app/(auth)/verify-email/page.tsx
- [ ] T038 [US2] Configure verification email template in Resend integration
- [ ] T039 [US2] Handle verification link callback and update user status

**Checkpoint**: Users can register and verify their email to gain chatbot access

---

## Phase 5: User Story 3 - User Login and Session Management (Priority: P2)

**Goal**: Allow verified users to log in and maintain sessions across browser sessions

**Independent Test**: Login, close browser, return, verify session persists via refresh token

### Implementation for User Story 3

- [ ] T040 [P] [US3] Create LoginForm component with validation in src/components/auth/LoginForm.tsx
- [ ] T041 [P] [US3] Create ForgotPasswordForm component in src/components/auth/ForgotPasswordForm.tsx
- [ ] T042 [US3] Create login page in src/app/(auth)/login/page.tsx
- [ ] T043 [US3] Create forgot password page in src/app/(auth)/forgot-password/page.tsx
- [ ] T044 [US3] Configure password reset email template in Resend
- [ ] T045 [US3] Add automatic session refresh handling in auth-client.ts
- [ ] T046 [US3] Create logout functionality with session invalidation

**Checkpoint**: Users can login, maintain sessions, and reset passwords

---

## Phase 6: User Story 1 - Reader Asks Question from Book Content (Priority: P1)

**Goal**: Verified users can ask questions and receive answers EXCLUSIVELY from book content

**Independent Test**: Ask about ROS 2 concepts, verify answer cites book sections, test "outside material" rejection

**Critical Constraints**:
- Similarity threshold: 0.7 - reject if no matching content
- Context-only responses - NO hallucination or external knowledge
- Rate limit: 10 messages/minute
- Max context: 2000 tokens

### Implementation for User Story 1

- [ ] T047 [P] [US1] Create ChatInput component with submit handling in src/components/chat/ChatInput.tsx
- [ ] T048 [P] [US1] Create ChatMessage component for user/assistant bubbles in src/components/chat/ChatMessage.tsx
- [ ] T049 [P] [US1] Create ChatHistory scrollable list component in src/components/chat/ChatHistory.tsx
- [ ] T050 [US1] Create ChatWidget embedded chat component in src/components/chat/ChatWidget.tsx
- [ ] T051 [US1] Create chat streaming hook for SSE in src/hooks/useChat.ts
- [ ] T052 [US1] Create POST /api/chat endpoint with RAG pipeline in src/app/api/chat/route.ts
- [ ] T053 [US1] Create POST /api/chat/context endpoint for selected text context in src/app/api/chat/context/route.ts
- [ ] T054 [US1] Implement email verification check in chat endpoints
- [ ] T055 [US1] Implement rate limiting check (10 msg/min) in chat endpoints
- [ ] T056 [US1] Implement "No matching answer found in book" response when similarity < 0.7
- [ ] T057 [US1] Implement "This question is outside the provided material" response for off-topic queries
- [ ] T058 [US1] Create standalone chat page in src/app/(main)/chat/page.tsx

**Checkpoint**: Core chatbot functional - answers ONLY from book content with proper guardrails

---

## Phase 7: User Story 5 - Conversation Logging for User History (Priority: P3)

**Goal**: Users can view their previous questions and answers

**Independent Test**: Ask multiple questions, navigate to history, verify all Q&A pairs displayed

### Implementation for User Story 5

- [ ] T059 [P] [US5] Create ConversationCard component in src/components/chat/ConversationCard.tsx
- [ ] T060 [P] [US5] Create ConversationList component in src/components/chat/ConversationList.tsx
- [ ] T061 [US5] Create GET /api/conversations endpoint with pagination in src/app/api/conversations/route.ts
- [ ] T062 [US5] Create GET /api/conversations/[id] endpoint in src/app/api/conversations/[id]/route.ts
- [ ] T063 [US5] Create DELETE /api/conversations/[id] endpoint in src/app/api/conversations/[id]/route.ts
- [ ] T064 [US5] Create conversation history page in src/app/(main)/history/page.tsx
- [ ] T065 [US5] Add conversation persistence to chat flow (save messages after each exchange)

**Checkpoint**: Users can view and manage their conversation history

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements across all user stories

- [ ] T066 [P] Add loading states and skeletons to chat and auth components
- [ ] T067 [P] Add error boundary components for graceful error handling
- [ ] T068 [P] Add toast notifications for success/error feedback
- [ ] T069 Implement CSP headers for XSS protection in next.config.js
- [ ] T070 Add request logging for debugging in production
- [ ] T071 Create seed script for admin user in scripts/seed-admin.ts
- [ ] T072 Validate quickstart.md flow with fresh setup
- [ ] T073 Run full integration test: signup → verify → login → ask question → view history

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories
- **Phase 3 (US4 - Ingest)**: Depends on Phase 2 - BLOCKS US1 (chatbot needs content)
- **Phase 4 (US2 - Signup)**: Depends on Phase 2 - Can run parallel with Phase 3
- **Phase 5 (US3 - Login)**: Depends on Phase 4 (signup required for login)
- **Phase 6 (US1 - Chat)**: Depends on Phase 3 (needs content) and Phase 4/5 (needs auth)
- **Phase 7 (US5 - History)**: Depends on Phase 6 (needs chat to create history)
- **Phase 8 (Polish)**: Depends on all user stories complete

### Critical Path

```
Phase 1 → Phase 2 → Phase 3 (Ingest Book) → Phase 6 (Chat) → Phase 7 (History)
                  ↘ Phase 4 (Signup) → Phase 5 (Login) ↗
```

### User Story Independence

- **US4 (Admin Ingest)**: Independent after Phase 2 - populates vector database
- **US2 (Signup)**: Independent after Phase 2 - creates user accounts
- **US3 (Login)**: Depends on US2 - needs users to login
- **US1 (Chat)**: Depends on US4 (content) + US2/US3 (auth) - core feature
- **US5 (History)**: Depends on US1 - needs conversations to display

### Parallel Opportunities

Within Phase 2:
- T012, T013, T014 (Qdrant) can run parallel with T015-T019 (Auth)
- T020-T025 (RAG Core) can run parallel with T015-T019 (Auth)

Within Phase 4:
- T033, T034, T035 can all run in parallel

Within Phase 6:
- T047, T048, T049 can all run in parallel

---

## Parallel Example: Phase 2 (Foundational)

```bash
# Launch database and vector DB setup in parallel:
Task: "Create Drizzle client in src/lib/db/index.ts"
Task: "Create Qdrant client in src/lib/qdrant.ts"

# Launch auth and RAG core in parallel:
Task: "Create Better Auth configuration in src/lib/auth.ts"
Task: "Create OpenAI embeddings service in src/lib/embeddings.ts"
```

---

## Implementation Strategy

### MVP First (Minimum Viable Chatbot)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL)
3. Complete Phase 3: US4 - Ingest ALL book content into Qdrant
4. Complete Phase 4: US2 - Enable user signup
5. Complete Phase 5: US3 - Enable user login
6. Complete Phase 6: US1 - Enable chatbot with book-only answers
7. **STOP and VALIDATE**: Verify chatbot answers ONLY from book content
8. Deploy as MVP

### Book Content Ingestion Priority

The chatbot MUST answer exclusively from book content. Ingestion order:
1. `chapters/01-ros2-nervous-system/` - ROS 2 fundamentals
2. `chapters/02-digital-twin/` - Digital twin concepts
3. `chapters/003-ai-robot-brain/` - AI robot brain content

### Book-Only Response Validation

Before deployment, verify these scenarios:
- [ ] Question about ROS 2 → Returns answer from book with chapter reference
- [ ] Question about weather → Returns "This question is outside the provided material"
- [ ] Question with no matching content (similarity < 0.7) → Returns "No matching answer found in book"
- [ ] Question spanning multiple chapters → Combines context from both
- [ ] 11th message in 1 minute → Rate limit error with retry time

---

## Task Summary

| Phase | Task Count | Purpose |
|-------|------------|---------|
| Phase 1: Setup | 7 | Project initialization |
| Phase 2: Foundational | 18 | Core infrastructure |
| Phase 3: US4 (Ingest) | 7 | Book content → Vector DB |
| Phase 4: US2 (Signup) | 7 | User registration |
| Phase 5: US3 (Login) | 7 | User authentication |
| Phase 6: US1 (Chat) | 12 | Chatbot with book-only answers |
| Phase 7: US5 (History) | 7 | Conversation management |
| Phase 8: Polish | 8 | Final improvements |
| **Total** | **73** | Complete feature |

---

## Notes

- [P] tasks = different files, no dependencies between them
- [USx] label maps task to specific user story
- Vector DB (Qdrant) stores ENTIRE book as embeddings - this is the chatbot's knowledge base
- Similarity threshold 0.7 ensures quality matches only
- System prompt enforces book-only responses - no hallucination
- Rate limiting protects against abuse (10 msg/min)
- Each user story is independently testable after completion
