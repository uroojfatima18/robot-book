# Implementation Plan: RAG Chatbot with Authentication

**Branch**: `006-rag-chatbot-auth` | **Date**: 2026-01-02 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-rag-chatbot-auth/spec.md`

## Summary

Implement an integrated RAG chatbot for the robotics book UI with secure authentication. Users can ask questions about book content and receive accurate, contextual answers derived exclusively from ingested material. The system includes:

- **Authentication**: Better Auth with email/password, JWT tokens, email verification via Resend
- **RAG Pipeline**: OpenAI embeddings → Qdrant vector search → Google Gemini streaming responses
- **Database**: Neon PostgreSQL with Drizzle ORM for users, sessions, conversations, rate limiting
- **Guardrails**: Similarity threshold (0.7), context-only responses, rate limiting (10 msg/min)

## Technical Context

**Language/Version**: TypeScript 5.x, Node.js 18+
**Primary Dependencies**: Next.js 14 (App Router), Better Auth, Drizzle ORM, @qdrant/js-client-rest, @google/genai, OpenAI SDK
**Storage**: Neon PostgreSQL (relational), Qdrant Cloud (vectors)
**Testing**: Vitest for unit/integration, Playwright for E2E
**Target Platform**: Vercel (serverless), Edge-compatible
**Project Type**: Web application (frontend + backend API routes)
**Performance Goals**: <5s response time (p95), 100 concurrent users
**Constraints**: <2000 token context, 10 msg/min rate limit, 0.7 similarity threshold
**Scale/Scope**: ~6 book chapters, ~1000 content chunks, 1000+ registered users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Embodied Learning | N/A | Chatbot feature, not chapter content |
| II. Simulation-First | N/A | Infrastructure feature |
| III. Agent-Human Partnership | PASS | Chatbot enables AI-assisted learning from book content |
| IV. Progressive Mastery | N/A | Not a chapter |
| V. AI-Native Content | PASS | RAG-compatible, text queryable via chatbot |
| VI. ROS 2 + Python | N/A | TypeScript web app (complementary to book) |
| VII. Safety & Ethics | PASS | No unsafe robot commands; strict context guardrails |

**Gate Result**: PASS - All applicable principles satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/006-rag-chatbot-auth/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 research findings
├── data-model.md        # Entity definitions and schema
├── quickstart.md        # Developer setup guide
├── contracts/
│   ├── openapi.yaml     # REST API specification
│   └── types.ts         # Shared TypeScript types
└── tasks.md             # Implementation tasks (from /sp.tasks)
```

### Source Code (repository root)

```text
src/
├── app/
│   ├── api/
│   │   ├── auth/[...all]/route.ts    # Better Auth handler
│   │   ├── chat/route.ts             # POST /api/chat (streaming)
│   │   ├── chat/context/route.ts     # POST /api/chat/context
│   │   ├── conversations/
│   │   │   ├── route.ts              # GET /api/conversations
│   │   │   └── [id]/route.ts         # GET/DELETE /api/conversations/:id
│   │   └── admin/
│   │       ├── ingest/route.ts       # POST /api/admin/ingest
│   │       └── stats/route.ts        # GET /api/admin/stats
│   ├── (auth)/
│   │   ├── login/page.tsx
│   │   ├── signup/page.tsx
│   │   ├── verify-email/page.tsx
│   │   └── forgot-password/page.tsx
│   └── (main)/
│       ├── chat/page.tsx             # Standalone chat interface
│       └── history/page.tsx          # Conversation history
├── components/
│   ├── chat/
│   │   ├── ChatWidget.tsx            # Embedded chat widget
│   │   ├── ChatInput.tsx             # Message input with submit
│   │   ├── ChatMessage.tsx           # Message bubble component
│   │   └── ChatHistory.tsx           # Scrollable message list
│   └── auth/
│       ├── LoginForm.tsx
│       ├── SignupForm.tsx
│       └── AuthProvider.tsx
├── lib/
│   ├── auth.ts                       # Better Auth configuration
│   ├── auth-client.ts                # Client-side auth helpers
│   ├── db/
│   │   ├── index.ts                  # Drizzle client
│   │   ├── schema.ts                 # Database schema
│   │   └── migrations/               # Drizzle migrations
│   ├── qdrant.ts                     # Qdrant client & helpers
│   ├── embeddings.ts                 # OpenAI embedding generation
│   ├── gemini.ts                     # Gemini chat client
│   ├── rag/
│   │   ├── retrieve.ts               # Vector retrieval logic
│   │   ├── chunk.ts                  # Text chunking utilities
│   │   └── truncate.ts               # Context truncation
│   └── rate-limit.ts                 # Rate limiting utilities
├── types/
│   └── index.ts                      # Re-export from contracts
└── middleware.ts                     # Auth middleware

tests/
├── unit/
│   ├── rag/
│   │   ├── chunk.test.ts
│   │   └── retrieve.test.ts
│   └── auth/
│       └── rate-limit.test.ts
├── integration/
│   ├── chat.test.ts
│   └── auth.test.ts
└── e2e/
    ├── signup-flow.spec.ts
    └── chat-flow.spec.ts
```

**Structure Decision**: Web application structure with Next.js App Router. API routes handle backend logic, components provide UI, lib contains shared utilities.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Client (Browser)                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐│
│  │ Auth Forms  │  │ Chat Widget │  │ Book Reader │  │ History View        ││
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘│
└─────────┼────────────────┼────────────────┼────────────────────┼───────────┘
          │                │                │                    │
          ▼                ▼                ▼                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Next.js API Routes (Vercel)                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐│
│  │ /api/auth/* │  │ /api/chat   │  │ /api/chat/  │  │ /api/conversations  ││
│  │ Better Auth │  │ (streaming) │  │   context   │  │     (history)       ││
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘│
│         │                └────────┬───────┘                    │           │
│         │                         ▼                            │           │
│         │         ┌───────────────────────────────┐            │           │
│         │         │         RAG Pipeline          │            │           │
│         │         │  ┌─────────┐   ┌───────────┐  │            │           │
│         │         │  │Embedding│──▶│  Qdrant   │  │            │           │
│         │         │  │ OpenAI  │   │  Search   │  │            │           │
│         │         │  └─────────┘   └─────┬─────┘  │            │           │
│         │         │                      ▼        │            │           │
│         │         │              ┌───────────┐    │            │           │
│         │         │              │  Gemini   │    │            │           │
│         │         │              │ Streaming │    │            │           │
│         │         │              └───────────┘    │            │           │
│         │         └───────────────────────────────┘            │           │
└─────────┼──────────────────────────────────────────────────────┼───────────┘
          │                                                      │
          ▼                                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            External Services                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐ │
│  │ Neon PostgreSQL │  │  Qdrant Cloud   │  │       Resend Email          │ │
│  │ Users, Sessions │  │  Vector Store   │  │    Verification Emails      │ │
│  │ Conversations   │  │  Book Chunks    │  │                             │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Request Flow: User Asks Question

```
1. User types question in ChatWidget
   │
2. POST /api/chat { message, conversationId? }
   │
3. Middleware: Verify JWT, check email_verified
   │
4. Rate Limit Check: check_rate_limit(user_id)
   │  └─ If exhausted → 429 { retryAfter: N }
   │
5. Generate Embedding: text-embedding-3-small(question)
   │
6. Vector Search: qdrant.search(embedding, limit=5, threshold=0.7)
   │  └─ If no results → Return "No matching answer found in book"
   │
7. Truncate Context: top chunks → max 2000 tokens
   │
8. Gemini Streaming: gemini.sendMessageStream(context, question)
   │
9. SSE Response: stream tokens to client
   │
10. Save to DB: Insert message to conversations/messages
```

## Component Dependencies

| Component | Depends On | Blocks |
|-----------|------------|--------|
| Database Schema | - | Auth, Chat, History |
| Better Auth Setup | Database Schema | All authenticated features |
| Qdrant Collection | - | RAG Pipeline |
| Embedding Service | OpenAI API | RAG Pipeline |
| RAG Pipeline | Qdrant, Embeddings, Gemini | Chat API |
| Chat API | Auth, RAG Pipeline, Rate Limit | Chat UI |
| Chat UI | Chat API | E2E Tests |
| Auth UI | Better Auth | User Flows |
| Admin Ingest | Auth (admin role), RAG | Content Population |

## Security Considerations

| Concern | Mitigation |
|---------|------------|
| Password storage | Better Auth uses bcrypt by default |
| JWT security | HTTP-only cookies, short-lived access tokens |
| Rate limiting | PostgreSQL token bucket (atomic) |
| Prompt injection | Context-only responses, system prompt isolation |
| XSS | React escaping, CSP headers |
| SQL injection | Drizzle parameterized queries |
| Admin access | Role-based authorization checks |

## Artifacts Generated

| Artifact | Path | Description |
|----------|------|-------------|
| Research | `specs/006-rag-chatbot-auth/research.md` | Technology decisions and patterns |
| Data Model | `specs/006-rag-chatbot-auth/data-model.md` | Entity definitions and schema |
| OpenAPI Spec | `specs/006-rag-chatbot-auth/contracts/openapi.yaml` | REST API contract |
| TypeScript Types | `specs/006-rag-chatbot-auth/contracts/types.ts` | Shared type definitions |
| Quickstart | `specs/006-rag-chatbot-auth/quickstart.md` | Developer setup guide |

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks from this plan
2. Review task order and dependencies
3. Begin implementation with database schema (blocking dependency)

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Gemini API rate limits | Medium | High | Implement retry with backoff, queue requests |
| Vector search latency | Low | Medium | Cache common queries, optimize Qdrant settings |
| Email deliverability | Medium | Medium | Monitor Resend metrics, add SPF/DKIM |
| Cold start latency | Medium | Low | Use edge functions where possible |

---

**Plan Status**: COMPLETE
**Ready for**: `/sp.tasks` command to generate implementation tasks
