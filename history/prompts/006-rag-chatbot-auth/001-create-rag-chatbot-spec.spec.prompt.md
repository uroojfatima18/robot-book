---
id: 001
title: Create RAG Chatbot Spec
stage: spec
date: 2026-01-02
surface: agent
model: claude-opus-4-5-20251101
feature: 006-rag-chatbot-auth
branch: 006-rag-chatbot-auth
user: urooj
command: /sp.specify
labels: ["rag", "chatbot", "authentication", "specification", "openai"]
links:
  spec: specs/006-rag-chatbot-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-rag-chatbot-auth/spec.md
  - specs/006-rag-chatbot-auth/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

```text
# Integrated RAG Chatbot + Signup System Technical Specifications

## Goal
A Retrieval-Augmented Generation (RAG) chatbot integrated into the robotics book UI that:
- Uses OpenAI Agents FastAPI
- Stores embeddings in Qdrant Cloud (Free Tier)
- Uses Neon Serverless PostgreSQL as the main database
- Answers questions only from selected text or book content
- Includes secure signup + authentication system

## Architecture Overview
| Component | Technology |
|-----------|-------------|
| Frontend | Book Web UI (Next.js / React) |
| Chatbot | OpenAI Agents SDK + FastAPI |
| Vector DB | Qdrant Cloud (RAG) |
| Main DB | Neon Serverless PostgreSQL |
| Auth | JWT + Refresh Tokens (Role-Based Access) |
| Storage | PostgreSQL (User Data, Logs), Qdrant (Embeddings) |

## Authentication & Signup Requirements
- Email-based signup (no social login initially)
- Password hashing using Argon2
- Token System: Access Token (JWT short-lived), Refresh Token (HTTP-only cookie, long-lived)
- Role-Based Access: user (read-only), admin (manage content)
- Block unverified accounts from chatbot usage
- Email verification required before chatbot access

## RAG Pipeline Requirements
Flow: User selects text -> backend -> cleaned/chunked (400-700 tokens) -> embeddings -> Qdrant storage with metadata -> query with cosine similarity/top-k=5 -> OpenAI agent for final answer

Rules:
- No matching context: respond "No matching answer found in book"
- No hallucination allowed
- Only answer from retrieved context

## Chatbot Guardrails
- Refuse answers outside book content
- Detect ambiguous questions and re-ask
- Auto-format: bullets, steps, code examples

## API Endpoints
POST /auth/signup, /auth/login, GET /auth/refresh, POST /rag/ingest (admin), /rag/query (protected), /chat (protected)

## Performance & Scaling
- Neon autoscaling, Qdrant HNSW, 10 msg/min rate limit, 2,000 token context limit

## UX Requirements
- Must be logged in to chat, unverified email shows message, text selection uses only that portion, streaming responses
```

## Response snapshot

Created feature specification `006-rag-chatbot-auth` for the Integrated RAG Chatbot with Authentication System:

**Branch**: `006-rag-chatbot-auth`
**Spec File**: `specs/006-rag-chatbot-auth/spec.md`

**Key deliverables**:
1. 5 prioritized user stories (P1-P3) with acceptance scenarios
2. 24 functional requirements across Authentication, RAG Pipeline, Chatbot Behavior, Logging, and Admin functions
3. 10 measurable success criteria (technology-agnostic)
4. 7 edge cases with expected behaviors
5. Clear Assumptions and Out of Scope sections
6. Quality checklist passed all validation items

The specification focuses on WHAT (user needs) not HOW (implementation), keeping technical stack details in the Input field for reference during planning.

## Outcome

- Impact: Created complete feature specification ready for planning phase
- Tests: N/A (specification phase)
- Files: 2 files created (spec.md, checklists/requirements.md)
- Next prompts: /sp.clarify or /sp.plan to proceed with implementation planning
- Reflection: Comprehensive spec generated from detailed user requirements; no clarifications needed as user provided thorough technical context

## Evaluation notes (flywheel)

- Failure modes observed: None - user provided complete technical requirements
- Graders run and results (PASS/FAIL): All checklist items PASS
- Prompt variant (if applicable): Standard /sp.specify with detailed user input
- Next experiment (smallest change to try): N/A - successful generation
