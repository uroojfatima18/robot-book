# Implementation Plan: RAG Chatbot Backend

**Branch**: `007-rag-chatbot-backend` | **Date**: 2026-02-22 | **Spec**: [spec.md](./spec.md)

---

## Summary

Minimal, correct RAG chatbot backend. Clean separation: retrieval module is independent from the agent module. Hallucination prevention is enforced by backend code (not LLM prompts). FastAPI serves `POST /ask` with retrieval-gating before any LLM call.

---

## Technical Stack

| Item | Decision | Rationale |
|------|----------|-----------|
| Language | Python 3.11+ | Async support, agents SDK |
| Framework | FastAPI | Async-first, auto docs |
| LLM | Gemini 2.0 Flash (free) via OpenAI Agents SDK | Per requirement |
| Embeddings | `text-embedding-3-small` via OpenAI | 1536 dims, low cost |
| Vector store | Qdrant Cloud Free Tier | ANN search, managed |
| Metadata store | Neon Serverless PostgreSQL | Chunk metadata, free tier |
| Agent SDK | `openai-agents` | Per requirement |
| Chunking | Heading-aware split (`#`/`##`) + frontmatter strip | Semantic context preservation |
| Config | `.env` + `python-dotenv` | Per FR-016 |

---

## Critical Design: Retrieval Gate

```
POST /ask
  ↓
retrieve_relevant_chunks(question)  ← embedding search in Qdrant
  ↓
if chunks == []:                    ← BACKEND CODE, not LLM
    return FALLBACK immediately
  ↓
context = "\n\n".join(chunk.text for chunk in chunks)
  ↓
ask_book(question, context)         ← LLM called ONLY with retrieved context
  ↓
return {"answer": ..., "sources": [...]}
```

This gate is the **primary hallucination prevention**. The LLM is never reached for off-topic questions.

---

## Folder Structure

```
backend/
├── .env                         # Secrets (gitignored)
├── requirements.txt             # All Python dependencies
├── ingest.py                    # Standalone ingestion script
└── app/
    ├── main.py                  # FastAPI app + CORS middleware
    ├── config.py                # Settings from .env
    ├── api/
    │   └── routes.py           # POST /ask, GET /health
    ├── agents/
    │   └── book_agent.py       # Gemini agent + ask_book()
    ├── rag/
    │   ├── chunker.py          # .md files → BookChunks
    │   ├── embedder.py         # text → list[float]
    │   └── retriever.py        # question → top-5 chunks (Qdrant)
    └── db/
        ├── qdrant_client.py    # Qdrant: ensure collection, upsert, search
        └── neon_client.py      # Neon: ensure table, upsert chunk
```

No extra files. No unnecessary abstraction layers.

---

## Implementation Phases

### Phase 1: Setup

- Create folder structure
- Write `requirements.txt`
- Install packages in `.venv`

### Phase 2: Config + DB Clients

- `config.py` — load and validate all env vars
- `db/qdrant_client.py` — connect, `ensure_collection()`, `upsert_chunks()`, `search_chunks()`
- `db/neon_client.py` — connect via asyncpg, `ensure_table()`, `upsert_chunk()`

### Phase 3: RAG Pipeline

- `rag/chunker.py` — walk `my-website/docs/`, strip frontmatter, split on headings, return `list[BookChunk]`
- `rag/embedder.py` — embed single text or batch via OpenAI API
- `rag/retriever.py` — embed question → Qdrant search → return top-5 chunks

### Phase 4: Agent

- `agents/book_agent.py` — Gemini + OpenAI Agents SDK setup, `ask_book(question, context)` async function

### Phase 5: API

- `api/routes.py` — `POST /ask` with **retrieval gate**, `GET /health`
- `app/main.py` — FastAPI app, CORS, include router, startup event

### Phase 6: Ingestion Script

- `ingest.py` — load chunks → embed → upsert Qdrant + Neon

### Phase 7: Testing

- Run `python ingest.py`, verify chunk counts
- Test `/ask` with book question → grounded answer
- Test `/ask` with off-topic question → exact fallback
- Test `/health` → all connected

---

## Deliverables Checklist

| Artifact | Status | Path |
|----------|--------|------|
| spec.md | ✅ Done | specs/007-rag-chatbot-backend/spec.md |
| plan.md | ✅ Done | specs/007-rag-chatbot-backend/plan.md |
| research.md | ✅ Done | specs/007-rag-chatbot-backend/research.md |
| data-model.md | ✅ Done | specs/007-rag-chatbot-backend/data-model.md |
| tasks.md | ✅ Done | specs/007-rag-chatbot-backend/tasks.md |
| backend/requirements.txt | ⬜ | backend/requirements.txt |
| backend/app/config.py | ⬜ | backend/app/config.py |
| backend/app/db/qdrant_client.py | ⬜ | backend/app/db/ |
| backend/app/db/neon_client.py | ⬜ | backend/app/db/ |
| backend/app/rag/chunker.py | ⬜ | backend/app/rag/ |
| backend/app/rag/embedder.py | ⬜ | backend/app/rag/ |
| backend/app/rag/retriever.py | ⬜ | backend/app/rag/ |
| backend/app/agents/book_agent.py | ⬜ | backend/app/agents/ |
| backend/app/api/routes.py | ⬜ | backend/app/api/ |
| backend/app/main.py | ⬜ | backend/app/main.py |
| backend/ingest.py | ⬜ | backend/ingest.py |

---

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| Gemini API rate limits | Generous free tier; slow ingest batch size if needed |
| Neon cold start latency | Acceptable on free tier; warm on first request |
| Embedding token limit (8192) | Cap each chunk at 2000 chars (~500 tokens) |
| Qdrant free tier 1-collection limit | One collection `book_chunks` only |
| CORS issues from GitHub Pages | `allow_origins=["*"]` for development; restrict in prod |
