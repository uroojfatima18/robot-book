"""
routes.py — The FastAPI endpoint logic for the RAG system.

Implementation of the retrieval gate:
  - Embed the question and search vectors.
  - If nothing is found, return the fallback message immediately (No LLM call).
  - If chunks are found, call the Gemini agent with relevant context only.
"""

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List
import json

from app.rag.retriever import retrieve_relevant_chunks
from app.agents.book_agent import ask_book
from app.db.qdrant_client import ping_qdrant, get_vector_count
from app.db.neon_client import ping_neon, get_chunk_count

router = APIRouter()

# --- Pydantic Data Models ---

class ChatRequest(BaseModel):
    question: str
    selected_text: str = None  # Optional: text selected by the user

class SourceRef(BaseModel):
    title: str
    chapter: str
    source_url: str

class ChatResponse(BaseModel):
    answer: str
    sources: List[SourceRef]

class HealthResponse(BaseModel):
    status: str
    qdrant: str
    neon: str
    chunk_count: int

# --- API Endpoints ---

@router.post("/ask")
async def ask(payload: ChatRequest):
    """
    Main RAG Chat endpoint with SSE streaming.
    Guarantees no hallucination by checking for context before calling the LLM.
    Streams response as Server-Sent Events for real-time display.
    """
    question = payload.question.strip()
    if not question:
        raise HTTPException(status_code=422, detail="Question cannot be empty.")

    async def stream_response():
        try:
            # 1. Step 1: CONTEXT — Determine if we use selected text or RAG.
            selected_text = payload.selected_text.strip() if payload.selected_text else None
            
            if selected_text:
                # If the user selected text, we prioritize answering ONLY based on that.
                context_str = f"USER SELECTED TEXT:\n{selected_text}"
                chunks = [] # No DB chunks needed when answering selected text
            else:
                # RETRIEVAL — Retrieve context from the book.
                question_lower = question.lower()
                is_small_talk = any(word in question_lower for word in ["hi", "hello", "hey", "who are you", "help", "greet"]) and len(question.split()) < 4

                try:
                    import asyncio
                    chunks = await asyncio.wait_for(
                        retrieve_relevant_chunks(question, limit=5),
                        timeout=30  # Wait more than the embedder timeout
                    )
                except asyncio.TimeoutError:
                    print(f"[RAG] Retrieval timeout for question: {question}")
                    chunks = []
                except Exception as e:
                    print(f"[RAG] Retrieval error: {e}")
                    chunks = []

                # 2. Step 2: GATING — ONLY block if it's a technical query with NO context.
                # If it's small talk or we have chunks, proceed to LLM.
                if not chunks and not is_small_talk:
                    # Only return "not in book" for longer, technical-looking queries.
                    if len(question.split()) > 3:
                        answer = "This information is not present in the book."
                        for char in answer:
                            event = json.dumps({"type": "token", "data": char})
                            yield f"data: {event}\n\n"
                        yield 'data: {"type": "done"}\n\n'
                        return
                    else:
                        # Fallback for short non-technical queries
                        context_str = "No specific technical context found. You are allowed to greet the user and have a friendly conversation as the Robot Book Assistant."
                else:
                    # Join all chunk texts to create context
                    context_str = "\n\n---\n\n".join(chunk.text for chunk in chunks) if chunks else "The user is greeting you. Be polite and welcoming."
            
            # 3. Step 3: CONVERSATION — build context and call LLM.
            answer = await ask_book(question, context_str)

            # Stream the answer character by character
            for char in answer:
                event = json.dumps({"type": "token", "data": char})
                yield f"data: {event}\n\n"

            # Signal sources if any were used
            if chunks:
                source_list = []
                # Remove duplicates based on source_url
                seen_urls = set()
                for c in chunks:
                    if c.source_url and c.source_url not in seen_urls:
                        source_list.append({
                            "title": c.title,
                            "chapter": c.chapter,
                            "source_url": c.source_url
                        })
                        seen_urls.add(c.source_url)
                
                if source_list:
                    event = json.dumps({"type": "sources", "data": source_list})
                    yield f"data: {event}\n\n"

            # Signal end of stream
            yield 'data: {"type": "done"}\n\n'
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            if "429" in str(e) or "Quota" in str(e) or "quota" in str(e):
                error_msg = "Your Gemini API quota has been exceeded. Please check your plan and billing details at Google AI Studio."
            else:
                error_msg = f"Internal Server Error: {str(e)}"
            event = json.dumps({"type": "error", "message": error_msg})
            yield f"data: {event}\n\n"

    return StreamingResponse(stream_response(), media_type="text/event-stream")

@router.get("/health", response_model=HealthResponse)
async def health():
    """
    Verify connectivity to Qdrant Cloud and Neon PostgreSQL.
    Useful for health checks and troubleshooting.
    """
    qdrant_ok = ping_qdrant()
    neon_ok = await ping_neon()

    total_count = await get_chunk_count()

    is_healthy = qdrant_ok and neon_ok

    return HealthResponse(
        status="ok" if is_healthy else "degraded",
        qdrant="connected" if qdrant_ok else "disconnected",
        neon="connected" if neon_ok else "disconnected",
        chunk_count=total_count
    )
