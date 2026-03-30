# Feature Specification: RAG Chatbot Backend (Book-Only AI Assistant)

**Feature Branch**: `007-rag-chatbot-backend`
**Created**: 2026-02-22
**Status**: Active
**Input**: Build a single RAG chatbot embedded in the Docusaurus book that answers questions ONLY from book content using FastAPI, OpenAI Agents SDK (Gemini), Qdrant, and Neon Postgres.

---

## Clarifications

### Session 2026-02-22

- Q: Should the chatbot answer questions outside the book? → A: **No.** Backend must return a fixed fallback **before calling the LLM** if no chunks are found.
- Q: Is hallucination prevention via prompt alone sufficient? → A: **No.** It must be enforced at backend logic level (retrieval-gating).
- Q: Can we pass the full book text to the model? → A: **Absolutely not.** RAG retrieval must always reduce context to only relevant chunks.
- Q: What is the API endpoint? → A: `POST /ask` — `{"question": string}` → `{"answer": string, "sources": []}`
- Q: What LLM? → A: Gemini 2.0 Flash via OpenAI Agents SDK (`openai-agents`)
- Q: What embedding model? → A: OpenAI `text-embedding-3-small`
- Q: What vector store? → A: Qdrant Cloud Free Tier
- Q: What metadata store? → A: Neon Serverless PostgreSQL
- Q: Selected-text Q&A? → A: **Out of scope.** Full-book chatbot only.
- Q: Streaming? → A: No. Simple request-response.
- Q: Authentication? → A: Not required for `/ask`.

---

## Overview

Build a minimal but correct RAG chatbot backend for the Physical AI & Humanoid Robotics book. The system:

1. **Ingests** all book Markdown files, chunks them, stores vectors in Qdrant and metadata in Neon.
2. **At query time**: retrieves top-K relevant chunks → if none found, returns fallback immediately (no LLM call) → builds a context-only prompt → calls the Gemini agent → returns the answer.
3. **Never hallucinations**: the backend logic gate prevents LLM calls when no relevant content exists.

---

## ⚠️ Non-Negotiable Rules

### Rule 1 — Backend-Level Hallucination Prevention (Critical)

The fallback is returned by **Python code**, NOT by the LLM:

```
FastAPI Route /ask
  ↓
retrieve_relevant_chunks(question) → list of chunks
  ↓
if len(chunks) == 0:
    return {"answer": "This information is not present in the book.", "sources": []}
    # ← LLM is NEVER called here
  ↓
context = build_context(chunks)
  ↓
answer = await ask_book(question, context)  # LLM called only with real context
  ↓
return {"answer": answer, "sources": [...chunk metadata...]}
```

### Rule 2 — No Full-Book Context Stuffing

The prompt sent to the LLM contains ONLY retrieved chunks, never the full book.

### Rule 3 — Retrieval Before LLM

`retrieve_relevant_chunks()` is always called first. The agent module never performs retrieval internally.

### Rule 4 — Exact Fallback Phrase

When no chunks found, response must be exactly:
```
"This information is not present in the book."
```

---

## System Architecture

```
[Docusaurus Book — GitHub Pages]
        │
        │ POST /ask {"question": "..."}
        ▼
[FastAPI Backend]
        │
        ▼
[Route Handler: /ask]
        │
        ├─→ retrieve_relevant_chunks(question)
        │       ├── embed question (OpenAI text-embedding-3-small)
        │       ├── search Qdrant → top-5 scored chunks
        │       └── returns: list[ChunkResult] or []
        │
        ├─→ if chunks == []:
        │       return FALLBACK (LLM never called)
        │
        ├─→ build_context(chunks) → context string
        │
        ├─→ ask_book(question, context) [OpenAI Agents SDK + Gemini]
        │       └── prompt = "Book Context:\n{context}\n\nQuestion:\n{question}"
        │
        └─→ return {"answer": ..., "sources": [...]}

[Qdrant Cloud]  ←── vectors + payloads (searched at query time, upserted at ingest)
[Neon Postgres] ←── chunk metadata (text, title, chapter, source_url)
```

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Reader Asks a Book Question (Priority: P1)

A reader types a robotics question. System retrieves relevant chunks, LLM answers from that context only.

**Independent Test**: `POST /ask {"question": "What is a ROS 2 node?"}` → answer contains the correct definition from the book.

**Acceptance Scenarios**:

1. **Given** a book-covered question, **When** chunks are found, **Then** the LLM is called with ONLY those chunks as context — answer is grounded in book.
2. **Given** an off-topic question, **When** NO chunks are found, **Then** backend returns `"This information is not present in the book."` WITHOUT calling the LLM.
3. **Given** any valid question, **When** the API is called, **Then** response arrives in < 8 seconds.

---

### User Story 2 — Admin Ingests Book Content (Priority: P2)

A developer runs `python ingest.py` to populate Qdrant + Neon from `my-website/docs/` Markdown files.

**Independent Test**: Run `python ingest.py` → Qdrant has vectors, Neon `chunks` table has matching rows.

**Acceptance Scenarios**:

1. **Given** all Markdown files in `my-website/docs/`, **When** ingest runs, **Then** all non-trivial chunks are stored in both Qdrant and Neon.
2. **Given** ingest has run before, **When** run again, **Then** no duplicates (upsert by UUID).
3. **Given** a Markdown file with YAML frontmatter, **When** ingested, **Then** frontmatter is stripped and not included in chunk text.

---

### User Story 3 — Health Check (Priority: P3)

**Independent Test**: `GET /health` → `{"status": "ok", "qdrant": "connected", "neon": "connected"}`

---

### Edge Cases

| Situation | Expected Behavior |
|-----------|------------------|
| Empty question string | Return HTTP 422: `"Question cannot be empty."` |
| No chunks retrieved | Return fallback immediately, no LLM call |
| Embedding API fails | Return HTTP 503, log error |
| Chunk < 20 words | Skip during ingestion |
| Re-run ingestion | Upsert, no duplication |

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST ingest all `.md` files from `my-website/docs/` recursively
- **FR-002**: Ingestion MUST strip YAML frontmatter (`---` blocks) before chunking
- **FR-003**: Ingestion MUST chunk by heading boundaries (split on `#`/`##`)
- **FR-004**: Each chunk MUST be embedded using OpenAI `text-embedding-3-small` (1536 dims)
- **FR-005**: Vectors MUST be stored in Qdrant Cloud, collection `book_chunks`
- **FR-006**: Chunk metadata (id, text, title, chapter, source_url) MUST be stored in Neon `chunks` table
- **FR-007**: Ingestion MUST upsert by UUID (no duplicates on re-run)
- **FR-008**: `POST /ask` MUST accept `{"question": string}`, return `{"answer": string, "sources": list}`
- **FR-009**: `POST /ask` MUST call `retrieve_relevant_chunks()` BEFORE any LLM interaction
- **FR-010**: If `retrieve_relevant_chunks()` returns empty list, MUST return fallback immediately — LLM MUST NOT be called
- **FR-011**: If chunks found, prompt sent to LLM MUST contain ONLY retrieved chunk text — no full book
- **FR-012**: LLM MUST be Gemini 2.0 Flash via OpenAI Agents SDK
- **FR-013**: Agent instructions MUST reinforce book-only answering (defense-in-depth)
- **FR-014**: `GET /health` MUST verify Qdrant + Neon connectivity
- **FR-015**: CORS MUST be enabled for GitHub Pages frontend origin
- **FR-016**: All secrets MUST be loaded from `.env` via `python-dotenv`
- **FR-017**: Retrieval and agent modules MUST be fully separated (no retrieval inside agent)

### Non-Functional Requirements

- **NFR-001**: Python 3.11+
- **NFR-002**: Async-first (FastAPI + asyncio)
- **NFR-003**: Qdrant Cloud Free Tier compatible
- **NFR-004**: Neon Serverless Postgres Free Tier
- **NFR-005**: Gemini Free API only for inference
- **NFR-006**: P95 response time < 8 seconds
- **NFR-007**: PEP 8 style

### Key Entities

- **BookChunk**: `{id: UUID, text: str, title: str, chapter: str, source_url: str, chunk_index: int}`
- **ChatRequest**: `{question: str}`
- **ChatResponse**: `{answer: str, sources: list[SourceRef]}`
- **SourceRef**: `{title: str, chapter: str, source_url: str}`

---

## Hallucination Prevention — Explained

Two layers of defense:

| Layer | Mechanism | Where |
|-------|-----------|-------|
| **Layer 1 (Primary)** | Backend code gates: if no chunks → return fallback, never call LLM | `api/routes.py` |
| **Layer 2 (Defense-in-depth)** | Agent instructions say "answer only from context" | `agents/book_agent.py` |

Layer 1 is the enforceable guarantee. Layer 2 is a reinforcement.

---

## Success Criteria *(mandatory)*

- **SC-001**: `POST /ask` with book-covered question → grounded, correct answer
- **SC-002**: `POST /ask` with off-topic question → exact fallback, no LLM called
- **SC-003**: `python ingest.py` processes all 4 chapters without error
- **SC-004**: P95 response time < 8 seconds for `/ask`
- **SC-005**: Re-running ingest creates no duplicate records
- **SC-006**: `GET /health` returns `200 OK` when all services connected

---

## Assumptions

- `my-website/docs/` contains finalized Markdown book content
- Qdrant Cloud collection `book_chunks` with 1536-dim COSINE vectors
- Neon PostgreSQL `chunks` table matches the schema in `data-model.md`
- Gemini and OpenAI API keys are present in `.env`
- Backend deployed separately from GitHub Pages (local or Render/Railway)

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `openai-agents` | OpenAI Agents SDK (Gemini integration) |
| `openai` | Embeddings API (`text-embedding-3-small`) |
| `fastapi` | Web framework |
| `uvicorn[standard]` | ASGI server |
| `qdrant-client[async]` | Qdrant Cloud client |
| `asyncpg` | Async PostgreSQL driver for Neon |
| `python-dotenv` | `.env` loading |
| `pydantic` | Request/response validation |

---

## Out of Scope

- Selected-text Q&A
- User authentication
- Streaming responses
- Chat history / memory
- Admin UI
- Rate limiting
- Multiple book collections
