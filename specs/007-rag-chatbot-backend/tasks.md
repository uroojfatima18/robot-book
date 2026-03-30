# Tasks: RAG Chatbot Backend

**Feature**: `007-rag-chatbot-backend` | **Date**: 2026-02-22
**Input**: spec.md, plan.md, research.md, data-model.md
**Constraint**: Minimal correct implementation — no over-engineering.

---

## Format

- `[P]` = Can run in parallel with other [P] tasks in the same phase
- All paths relative to `c:\Users\dell\Desktop\robot-book\`

---

## Phase 1: Setup

**Goal**: Create folder structure, write requirements, install packages.

- [ ] **T001** Create backend folder structure:
  ```
  backend/app/__init__.py
  backend/app/api/__init__.py
  backend/app/agents/__init__.py
  backend/app/rag/__init__.py
  backend/app/db/__init__.py
  ```

- [ ] **T002** Create `backend/requirements.txt`:
  ```
  openai-agents
  openai
  fastapi
  uvicorn[standard]
  qdrant-client[async]
  asyncpg
  python-dotenv
  pydantic
  ```

- [ ] **T003** Install: `cd backend && .venv\Scripts\pip install -r requirements.txt`

- [ ] **T004** Create `backend/.env` with all required variables (copy from root `.env`):
  ```
  GEMINI_API_KEY=...
  OPENAI_API_KEY=...
  QDRANT_URL=...
  QDRANT_API_KEY=...
  DATABASE_URL=...
  DOCS_PATH=../my-website/docs
  FRONTEND_URL=*
  ```

**✅ Checkpoint**: `pip list` shows all packages installed.

---

## Phase 2: Config + DB Clients

**Goal**: Load env vars, connect to Qdrant and Neon.

- [ ] **T005** Create `backend/app/config.py`:
  - Load all env vars using `load_dotenv`
  - Export as module-level constants (GEMINI_API_KEY, OPENAI_API_KEY, QDRANT_URL, etc.)
  - Raise `ValueError` on startup if any required var is missing

- [ ] **T006** [P] Create `backend/app/db/qdrant_client.py`:
  - `AsyncQdrantClient` initialized with `QDRANT_URL` and `QDRANT_API_KEY`
  - Constants: `COLLECTION = "book_chunks"`, `VECTOR_SIZE = 1536`
  - `async def ensure_collection()` — check if collection exists, create if not (COSINE, 1536)
  - `async def upsert_chunks(points: list[PointStruct])` — batch upsert
  - `async def search_chunks(vector: list[float], limit: int = 5) -> list` — return scored points

- [ ] **T007** [P] Create `backend/app/db/neon_client.py`:
  - Lazy `asyncpg` connection pool via `get_pool()`
  - `async def ensure_table()` — CREATE TABLE IF NOT EXISTS `chunks` per data-model.md
  - `async def upsert_chunk(chunk: BookChunk)` — INSERT ON CONFLICT DO UPDATE
  - `async def get_chunk_count() -> int`

**✅ Checkpoint**: Can call `ensure_collection()` and `ensure_table()` without error.

---

## Phase 3: RAG Pipeline

**Goal**: Build chunker → embedder → retriever. Each is an independent module.

- [ ] **T008** Create `backend/app/rag/chunker.py`:
  - `def strip_frontmatter(content: str) -> str` — removes `---` YAML blocks
  - `def chunk_markdown(content: str, source_url: str, chapter: str) -> list[BookChunk]`:
    - Split on `\n(?=#{1,2} )` regex
    - Skip chunks with < 20 words
    - Cap each chunk text at 2000 chars
    - Assign `uuid5(NAMESPACE_URL, f"{source_url}#{i}")` as ID
    - Extract title from first heading line
  - `def load_all_chunks(docs_path: str) -> list[BookChunk]`:
    - Walk `docs_path` recursively for `*.md` files
    - Extract chapter from folder name (e.g., `01-ros2-nervous-system`)
    - Return all chunks from all files

- [ ] **T009** [P] Create `backend/app/rag/embedder.py`:
  - `AsyncOpenAI` client with `OPENAI_API_KEY`
  - `async def embed_text(text: str) -> list[float]` — `text-embedding-3-small`, returns 1536-dim vector

- [ ] **T010** [P] Create `backend/app/rag/retriever.py`:
  - Import `embed_text` from embedder, `search_chunks` from qdrant_client
  - `async def retrieve_relevant_chunks(question: str, limit: int = 5) -> list[BookChunk]`:
    - Embed the question
    - Call `search_chunks(vector, limit)`
    - Convert Qdrant hits to `BookChunk` objects from payload
    - Return list (empty if no hits above threshold)

**✅ Checkpoint**: `retrieve_relevant_chunks("What is ROS 2?")` returns at least 1 result (after ingest).

---

## Phase 4: Agent

**Goal**: Wire Gemini via OpenAI Agents SDK. Agent answers ONLY from provided context.

- [ ] **T011** Create `backend/app/agents/book_agent.py`:
  - `AsyncOpenAI` client → Gemini endpoint
  - `OpenAIChatCompletionsModel("gemini-2.0-flash", openai_client=...)`
  - `RunConfig(model=model, model_provider=client)`
  - `Agent(name="BookOnlyAgent", instructions=..., model=model)` with strict book-only instruction
  - `async def ask_book(question: str, context: str) -> str`:
    - Builds prompt: `f"Book Context:\n{context}\n\nQuestion:\n{question}"`
    - Calls `Runner.run(book_agent, input=prompt, run_config=run_config)`
    - Returns `result.final_output`
  - **This function is ONLY called from routes.py after chunks are found**

**✅ Checkpoint**: `ask_book("What is ROS 2?", sample_text)` returns a string answer.

---

## Phase 5: API Layer

**Goal**: Implement the retrieval gate + API endpoints.

- [ ] **T012** Create `backend/app/api/routes.py`:

  ```python
  @router.post("/ask", response_model=ChatResponse)
  async def ask(payload: ChatRequest):
      # Validate
      if not payload.question.strip():
          raise HTTPException(422, "Question cannot be empty.")

      # 1. Retrieve FIRST — always
      chunks = await retrieve_relevant_chunks(payload.question)

      # 2. GATE — no chunks → return fallback (LLM NOT called)
      if not chunks:
          return ChatResponse(
              answer="This information is not present in the book.",
              sources=[]
          )

      # 3. Build context from chunks ONLY
      context = "\n\n---\n\n".join(c.text for c in chunks)

      # 4. Call LLM with retrieved context only
      answer = await ask_book(payload.question, context)

      sources = [SourceRef(title=c.title, chapter=c.chapter, source_url=c.source_url)
                 for c in chunks]
      return ChatResponse(answer=answer, sources=sources)

  @router.get("/health")
  async def health():
      # Ping Qdrant and Neon, return status dict
      ...
  ```

- [ ] **T013** Create `backend/app/main.py`:
  - `FastAPI(title="Book RAG Chatbot API", version="1.0.0")`
  - `CORSMiddleware` with `allow_origins=[FRONTEND_URL]`
  - `@app.on_event("startup")` → call `ensure_collection()` + `ensure_table()`
  - `app.include_router(router)`

**✅ Checkpoint**: `uvicorn app.main:app --reload` starts. `/docs` loads. `/health` returns 200.

---

## Phase 6: Ingestion Script

**Goal**: Populate Qdrant + Neon with all book content.

- [ ] **T014** Create `backend/ingest.py`:
  ```python
  # Standalone — run directly: python ingest.py
  import asyncio
  from dotenv import load_dotenv
  load_dotenv()

  async def main():
      await ensure_collection()
      await ensure_table()

      chunks = load_all_chunks(DOCS_PATH)
      print(f"Found {len(chunks)} chunks")

      for i, chunk in enumerate(chunks):
          vector = await embed_text(chunk.text)

          await upsert_chunks([PointStruct(
              id=chunk.id, vector=vector,
              payload={"text": chunk.text, "title": chunk.title,
                       "chapter": chunk.chapter, "source_url": chunk.source_url,
                       "chunk_index": chunk.chunk_index}
          )])
          await upsert_chunk(chunk)

          print(f"  [{i+1}/{len(chunks)}] {chunk.title[:60]}")

      print(f"\nDone. {len(chunks)} chunks ingested into Qdrant + Neon.")

  asyncio.run(main())
  ```

**✅ Checkpoint**: Script runs to completion. Qdrant dashboard shows vectors in `book_chunks`. Neon shows rows in `chunks`.

---

## Phase 7: Integration Testing

**Goal**: Verify the full pipeline end-to-end.

- [ ] **T015** Run `python ingest.py` → verify chunk count > 0

- [ ] **T016** [P] Test book question:
  ```bash
  curl -X POST http://localhost:8000/ask \
    -H "Content-Type: application/json" \
    -d "{\"question\": \"What is a ROS 2 node?\"}"
  ```
  → `answer` must contain relevant book content, `sources` must not be empty

- [ ] **T017** [P] Test off-topic question:
  ```bash
  curl -X POST http://localhost:8000/ask \
    -H "Content-Type: application/json" \
    -d "{\"question\": \"What is the recipe for pizza?\"}"
  ```
  → `answer` must be exactly `"This information is not present in the book."`, `sources` must be `[]`

- [ ] **T018** [P] Test empty question:
  ```bash
  curl -X POST http://localhost:8000/ask \
    -H "Content-Type: application/json" \
    -d "{\"question\": \"\"}"
  ```
  → HTTP 422

- [ ] **T019** Test health:
  ```bash
  curl http://localhost:8000/health
  ```
  → `{"status": "ok", "qdrant": "connected", "neon": "connected"}`

- [ ] **T020** [P] Run `python ingest.py` again → verify chunk count stays the same (no duplicates)

**✅ Checkpoint**: All 5 tests pass. Backend is fully functional.

---

## Phase 8: Frontend Widget (Docusaurus)

**Goal**: Embed the chatbot in the Docusaurus site.

- [ ] **T021** Create `my-website/src/components/ChatWidget.js`:
  - Floating chat button (bottom-right)
  - Chat panel with input + Send button
  - Calls `POST http://localhost:8000/ask` (env-configurable)
  - Shows answer + source chapter names
  - Loading state while waiting

- [ ] **T022** Import `<ChatWidget />` in `my-website/src/theme/Root.js` or `index.js` (global placement)

- [ ] **T023** Test in browser — verify chat works on the Docusaurus dev site

**✅ Checkpoint**: Chat bubble appears on the book site, answers book questions correctly.

---

## Execution Order

```
T001 → T002 → T003 → T004            (Phase 1, sequential)
  → T005 → [T006 ‖ T007]             (Phase 2, T006+T007 parallel)
  → [T008 ‖ T009 ‖ T010]             (Phase 3, all parallel after T005)
  → T011                              (Phase 4)
  → [T012 ‖ T013]                     (Phase 5, parallel)
  → T014                              (Phase 6)
  → T015 → [T016 ‖ T017 ‖ T018 ‖ T019] → T020   (Phase 7)
  → T021 → T022 → T023               (Phase 8)
```

---

## Summary

| Phase | Tasks | Notes |
|-------|-------|-------|
| 1: Setup | T001–T004 | Sequential |
| 2: DB Clients | T005–T007 | T006+T007 parallel |
| 3: RAG Pipeline | T008–T010 | All 3 parallel |
| 4: Agent | T011 | Single task |
| 5: API Layer | T012–T013 | Parallel |
| 6: Ingest Script | T014 | Single task |
| 7: Testing | T015–T020 | T016–T019 parallel |
| 8: Frontend | T021–T023 | Sequential |

**Total**: 23 tasks | **MVP** (backend only): Phases 1–7 (20 tasks)
