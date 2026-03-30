# Data Model: RAG Chatbot Backend

**Feature**: `007-rag-chatbot-backend` | **Date**: 2026-02-22

---

## 1. Neon PostgreSQL Schema

```sql
CREATE TABLE IF NOT EXISTS chunks (
    id           UUID         PRIMARY KEY,
    text         TEXT         NOT NULL,
    title        VARCHAR(500),
    chapter      VARCHAR(200),
    source_url   VARCHAR(500),
    chunk_index  INTEGER      NOT NULL DEFAULT 0,
    created_at   TIMESTAMPTZ  DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_chunks_chapter ON chunks(chapter);
```

| Column | Type | Description |
|--------|------|-------------|
| `id` | UUID | Primary key — same UUID as Qdrant point ID (deterministic via uuid5) |
| `text` | TEXT | Raw chunk text (max 2000 chars) |
| `title` | VARCHAR(500) | Section heading extracted from Markdown |
| `chapter` | VARCHAR(200) | Folder name, e.g. `01-ros2-nervous-system` |
| `source_url` | VARCHAR(500) | Relative `.md` file path |
| `chunk_index` | INTEGER | Position within source file |
| `created_at` | TIMESTAMPTZ | Insert timestamp |

---

## 2. Qdrant Vector Store

```
Collection : book_chunks
Vector Size: 1536
Distance   : COSINE
Point ID   : UUID string (matches Neon chunks.id)
```

### Point Payload

```json
{
  "text": "ROS 2 topics are named buses...",
  "title": "Topics and Nodes",
  "chapter": "01-ros2-nervous-system",
  "source_url": "docs/01-ros2-nervous-system/beginner/01-intro.md",
  "chunk_index": 3
}
```

**Both Qdrant payload and Neon store chunk text** — Qdrant payload is used at query time (avoids a DB join). Neon is the canonical record.

---

## 3. API Schemas

### `POST /ask`

**Request**:
```json
{ "question": "What is a ROS 2 node?" }
```

**Response — answer found** (200):
```json
{
  "answer": "A ROS 2 node is the basic executable unit in the ROS 2 ecosystem...",
  "sources": [
    {
      "title": "Introduction to ROS 2 Nodes",
      "chapter": "01-ros2-nervous-system",
      "source_url": "docs/01-ros2-nervous-system/beginner/01-intro.md"
    }
  ]
}
```

**Response — not in book** (200):
```json
{
  "answer": "This information is not present in the book.",
  "sources": []
}
```

**Response — validation error** (422):
```json
{ "detail": "Question cannot be empty." }
```

---

### `GET /health`

**Response — healthy** (200):
```json
{
  "status": "ok",
  "qdrant": "connected",
  "neon": "connected",
  "chunk_count": 247
}
```

**Response — degraded** (503):
```json
{
  "status": "degraded",
  "qdrant": "error: timeout",
  "neon": "connected"
}
```

---

## 4. Python Pydantic Models

```python
from pydantic import BaseModel
from uuid import UUID

# Ingestion internal model
class BookChunk(BaseModel):
    id: str          # uuid5 string
    text: str
    title: str
    chapter: str
    source_url: str
    chunk_index: int

# API models
class ChatRequest(BaseModel):
    question: str

class SourceRef(BaseModel):
    title: str
    chapter: str
    source_url: str

class ChatResponse(BaseModel):
    answer: str
    sources: list[SourceRef]

class HealthResponse(BaseModel):
    status: str
    qdrant: str
    neon: str
    chunk_count: int = 0
```

---

## 5. Environment Variables

| Variable | Used In | Example |
|----------|---------|---------|
| `GEMINI_API_KEY` | `agents/book_agent.py` | `AIzaSy...` |
| `OPENAI_API_KEY` | `rag/embedder.py` | `sk-proj-...` |
| `QDRANT_URL` | `db/qdrant_client.py` | `https://xxx.qdrant.io` |
| `QDRANT_API_KEY` | `db/qdrant_client.py` | `eyJhbG...` |
| `DATABASE_URL` | `db/neon_client.py` | `postgresql://...neon.tech/neondb?sslmode=require` |
| `DOCS_PATH` | `rag/chunker.py` | `../my-website/docs` |
| `FRONTEND_URL` | `app/main.py` (CORS) | `https://uroojfatima18.github.io` |
