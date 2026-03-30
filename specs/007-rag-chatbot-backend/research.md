# Research: RAG Chatbot Backend

**Feature**: `007-rag-chatbot-backend` | **Date**: 2026-02-22

---

## 1. OpenAI Agents SDK + Gemini — Verified Pattern

The `openai-agents` package accepts any OpenAI-compatible REST API. Gemini exposes one at `https://generativelanguage.googleapis.com/v1beta/openai/`.

```python
from agents import Agent, OpenAIChatCompletionsModel, RunConfig, AsyncOpenAI, Runner
from dotenv import load_dotenv, find_dotenv
import os

load_dotenv(find_dotenv())

external_client = AsyncOpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=external_client,
)

run_config = RunConfig(
    model=model,
    model_provider=external_client,
)

book_agent = Agent(
    name="BookOnlyAgent",
    instructions="""
You are a book assistant for the Physical AI & Humanoid Robotics book.
Answer ONLY using the provided Book Context below.
If the answer is not found in the Book Context, respond EXACTLY with:
"This information is not present in the book."
Do NOT use any outside knowledge. Do NOT make up answers.
""",
    model=model,
)

async def ask_book(question: str, context: str) -> str:
    """
    Call the book agent with retrieved context only.
    This function is only called AFTER retrieve_relevant_chunks() returns results.
    """
    prompt = f"""Book Context:
{context}

Question:
{question}"""

    result = await Runner.run(
        book_agent,
        input=prompt,
        run_config=run_config,
    )
    return result.final_output
```

**Key**: `ask_book()` is only called from the route handler AFTER the retrieval gate passes.

---

## 2. The Retrieval Gate — How Hallucination is Prevented at Code Level

```python
# api/routes.py  ← This is where the guarantee lives
@router.post("/ask")
async def ask(payload: ChatRequest):
    if not payload.question.strip():
        raise HTTPException(status_code=422, detail="Question cannot be empty.")

    # Step 1: Retrieve FIRST — always
    chunks = await retrieve_relevant_chunks(payload.question)

    # Step 2: GATE — if no chunks, return fallback WITHOUT calling LLM
    if not chunks:
        return ChatResponse(
            answer="This information is not present in the book.",
            sources=[]
        )

    # Step 3: Build context from retrieved chunks ONLY
    context = "\n\n---\n\n".join(c.text for c in chunks)

    # Step 4: Call LLM with retrieved context only
    answer = await ask_book(payload.question, context)

    sources = [SourceRef(title=c.title, chapter=c.chapter, source_url=c.source_url)
               for c in chunks]

    return ChatResponse(answer=answer, sources=sources)
```

This is a **hard code-level gate**. The LLM is literally unreachable for off-topic questions.

---

## 3. Qdrant Cloud — Async Client Pattern

```python
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

client = AsyncQdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

COLLECTION = "book_chunks"
VECTOR_SIZE = 1536  # text-embedding-3-small

async def ensure_collection():
    existing = await client.get_collections()
    names = [c.name for c in existing.collections]
    if COLLECTION not in names:
        await client.create_collection(
            collection_name=COLLECTION,
            vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE),
        )

async def upsert_chunks(points: list[PointStruct]):
    await client.upsert(collection_name=COLLECTION, points=points)

async def search_chunks(vector: list[float], limit: int = 5):
    return await client.search(
        collection_name=COLLECTION,
        query_vector=vector,
        limit=limit,
        with_payload=True,
    )
```

---

## 4. Neon PostgreSQL — asyncpg Pattern

```python
import asyncpg, os

_pool = None

async def get_pool():
    global _pool
    if _pool is None:
        _pool = await asyncpg.create_pool(dsn=os.getenv("DATABASE_URL"))
    return _pool

async def ensure_table():
    pool = await get_pool()
    async with pool.acquire() as conn:
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chunks (
                id          UUID PRIMARY KEY,
                text        TEXT NOT NULL,
                title       VARCHAR(500),
                chapter     VARCHAR(200),
                source_url  VARCHAR(500),
                chunk_index INTEGER NOT NULL DEFAULT 0,
                created_at  TIMESTAMPTZ DEFAULT NOW()
            )
        """)

async def upsert_chunk(chunk):
    pool = await get_pool()
    async with pool.acquire() as conn:
        await conn.execute("""
            INSERT INTO chunks (id, text, title, chapter, source_url, chunk_index)
            VALUES ($1, $2, $3, $4, $5, $6)
            ON CONFLICT (id) DO UPDATE SET
                text = EXCLUDED.text,
                title = EXCLUDED.title,
                chapter = EXCLUDED.chapter,
                source_url = EXCLUDED.source_url,
                chunk_index = EXCLUDED.chunk_index
        """, chunk.id, chunk.text, chunk.title, chunk.chapter, chunk.source_url, chunk.chunk_index)
```

---

## 5. Heading-Aware Markdown Chunker + Frontmatter Strip

```python
import re, os
from pathlib import Path
from uuid import uuid5, NAMESPACE_URL

def strip_frontmatter(content: str) -> str:
    """Remove YAML --- blocks from top of markdown."""
    if content.startswith("---"):
        end = content.find("---", 3)
        if end != -1:
            return content[end + 3:].strip()
    return content.strip()

def chunk_markdown(content: str, source_url: str, chapter: str) -> list[dict]:
    content = strip_frontmatter(content)
    # Split on H1 or H2 headings
    sections = re.split(r'\n(?=#{1,2} )', content)
    chunks = []
    for i, section in enumerate(sections):
        text = section.strip()
        if len(text.split()) < 20:   # skip trivial sections
            continue
        text = text[:2000]           # cap at ~500 tokens
        lines = text.split('\n')
        title = lines[0].lstrip('#').strip() if lines[0].startswith('#') else source_url
        # Deterministic UUID from source + index
        chunk_id = str(uuid5(NAMESPACE_URL, f"{source_url}#{i}"))
        chunks.append({
            "id": chunk_id,
            "text": text,
            "title": title,
            "chapter": chapter,
            "source_url": source_url,
            "chunk_index": i,
        })
    return chunks
```

**Why `uuid5`?** Deterministic UUIDs mean re-ingestion hits the same UUID → upsert works correctly, no duplicates.

---

## 6. OpenAI Embeddings

```python
from openai import AsyncOpenAI
import os

embed_client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

async def embed_text(text: str) -> list[float]:
    resp = await embed_client.embeddings.create(
        model="text-embedding-3-small",
        input=text,
    )
    return resp.data[0].embedding  # len=1536
```

**Cost estimate**: ~$0.002 total for all 4 book chapters (~100K tokens). Effectively free.

---

## 7. FastAPI CORS

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],          # Restrict to GitHub Pages URL in production
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)
```

---

## 8. Decisions Summary

| Decision | Chosen | Why |
|----------|--------|-----|
| Hallucination prevention | Backend code gate | Reliable; LLM can't override Python code |
| Embedding | OpenAI text-embedding-3-small | High quality, cheap, 1536 dims |
| LLM | Gemini 2.0 Flash | Free, OpenAI-compatible |
| Chunk UUID | uuid5 (deterministic) | Enables idempotent upsert |
| Async DB | asyncpg | Native async, no thread overhead |
| Chunk size cap | 2000 chars | Stays inside embedding token limit |
| Retrieval top-k | 5 | Enough context, not too long |
