
# Unified Book RAG Chatbot (Single Chatbot Mode)
## 🎯 Objective

Develop a single RAG-powered chatbot embedded in a Docusaurus book that:

- Answers questions only from book content
- Uses Spec-Kit Plus for spec-driven backend
- Uses OpenAI Agents SDK with Gemini Free API
- Runs on FastAPI
- Uses Qdrant Cloud for vectors
- Uses Neon Serverless Postgres for metadata
- Is deployed independently from GitHub Pages frontend

## 🧠 Scope (Updated)

- ❌ Selected text Q&A → REMOVED
- ✅ Full-book RAG chatbot → ENABLED

## 🏗️ System Architecture
```
[Docusaurus Book (GitHub Pages)]
        ↓
[Single Chat Widget]
        ↓
[FastAPI Backend]
        ↓
[RAG Pipeline]
 ├── Qdrant Cloud (Embeddings)
 ├── Neon Postgres (Chunk metadata)
 └── Gemini (OpenAI Agents SDK)
```

## 🧩 Functional Requirements
### FR-1: Book Content Ingestion

- Read Markdown files from Docusaurus /docs
- Chunk content (heading-aware)
- Generate embeddings
- Store:
    - Vectors → Qdrant
    - Metadata → Neon (page, section, url)

### FR-2: RAG-Based Question Answering

- Accept user question
- Retrieve top-k relevant chunks from Qdrant
- Pass retrieved content as context
- Generate answer using Gemini
- No hallucination allowed

### FR-3: Safe Answering Rule

If answer is not found in retrieved context, respond with:

"This information is not present in the book."

### FR-4: API Endpoints (Final)

| Method | Endpoint        | Purpose                 |
|--------|-----------------|-------------------------|
| POST   | /ask            | Ask question from book  |
| POST   | /ingest         | Ingest / update book content |
| GET    | /health         | Health check            |

## ⚙️ Non-Functional Requirements

- Python 3.10+
- Async-first
- Free-tier compatible
- .env based secrets
- Spec-Kit Plus compliant

## 📁 Folder Structure
```
backend/
├── spec/
│   └── book_rag.spec.md
├── app/
│   ├── main.py
│   ├── api/
│   │   └── routes.py
│   ├── agents/
│   │   └── book_agent.py
│   ├── rag/
│   │   ├── retriever.py
│   │   ├── embedder.py
│   │   └── chunker.py
│   ├── db/
│   │   ├── qdrant.py
│   │   └── neon.py
│   └── config.py
├── requirements.txt
└── .env
```

## 🤖 Gemini + OpenAI Agents SDK (Final Setup)
```python
from agents import (
    Agent,
    OpenAIChatCompletionsModel,
    RunConfig,
    AsyncOpenAI,
    Runner
)

from dotenv import load_dotenv, find_dotenv
import os

load_dotenv(find_dotenv())

gemini_api_key = os.getenv("GEMINI_API_KEY")

external_client = AsyncOpenAI(
    api_key=gemini_api_key,
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
```

## 🧠 Book RAG Agent (Strict)
```python
book_agent = Agent(
    name="BookOnlyAgent",
    instructions="""
You are a book assistant.
Answer ONLY using the provided book context.
If the answer is not found, say exactly:
"This information is not present in the book."
""",
    model=model,
)
```

## 🔎 RAG Ask Flow (Single Mode)
```python
async def ask_book(question: str, retrieved_chunks: str):
    prompt = f"""
Book Context:
{retrieved_chunks}

Question:
{question}
"""

    result = await Runner.run(
        book_agent,
        input=prompt,
        run_config=run_config,
    )

    return result.final_output
```

## 🌐 FastAPI Route (Final)
```python
from fastapi import APIRouter
from app.rag.retriever import retrieve_chunks
from app.agents.book_agent import ask_book

router = APIRouter()

@router.post("/ask")
async def ask(payload: dict):
    question = payload["question"]

    context = await retrieve_chunks(question)

    answer = await ask_book(question, context)

    return {
        "question": question,
        "answer": answer
    }
```

## 🖥️ Frontend (Docusaurus)

- One chatbot
- Simple input box
- Calls /ask
- Displays answer
- No text selection logic

## ✅ Final Result

- ✔ Single chatbot
- ✔ Book-only answers
- ✔ Gemini Free API
- ✔ Spec-Kit Plus compliant
- ✔ Clean & simple UX
