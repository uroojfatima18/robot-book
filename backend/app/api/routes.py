"""
routes.py — The FastAPI endpoint logic for the RAG system.

Implementation of the retrieval gate:
  - Embed the question and search vectors.
  - If nothing is found, return the fallback message immediately (No LLM call).
  - If chunks are found, call the Gemini agent with relevant context only.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List

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

@router.post("/ask", response_model=ChatResponse)
async def ask(payload: ChatRequest):
    """
    Main RAG Chat endpoint. 
    Guarantees no hallucination by checking for context before calling the LLM.
    """
    question = payload.question.strip()
    if not question:
        raise HTTPException(status_code=422, detail="Question cannot be empty.")

    try:
        # 1. Step 1: CONTEXT — Determine if we use selected text or RAG.
        selected_text = payload.selected_text.strip() if payload.selected_text else None
        
        if selected_text:
            # If the user selected text, we prioritize answering ONLY based on that.
            context_str = f"USER SELECTED TEXT:\n{selected_text}"
            chunks = [] # No DB chunks needed when answering selected text
        else:
            # RETRIEVAL — Retrieve FIRST, always.
            # This reduces the context to only relevant parts of the book.
            chunks = await retrieve_relevant_chunks(question, limit=5)

            # 2. Step 2: GATING — Hallucination prevention at the code level.
            # If no relevant chunks are found, we return the fallback message IMMEDIATELY.
            # The expensive (and potentially hallucinatory) LLM is NEVER called in this case.
            if not chunks:
                return ChatResponse(
                    answer="This information is not present in the book.",
                    sources=[]
                )

            # Join all chunk texts to create a focused pool of knowledge for the model.
            context_str = "\n\n---\n\n".join(chunk.text for chunk in chunks)
        
        # 3. Step 3: CONVERSATION — build context and call LLM.
        # We only pass the focused context, not the whole book.
        answer = await ask_book(question, context_str)

        # 4. Step 4: SOURCES — Format the response with source metadata for the reader.
        sources = [
            SourceRef(title=c.title, chapter=c.chapter, source_url=c.source_url)
            for c in chunks
        ]

        return ChatResponse(
            answer=answer,
            sources=sources
        )
    except Exception as e:
        import traceback
        traceback.print_exc()
        if "429" in str(e) or "Quota" in str(e) or "quota" in str(e):
            raise HTTPException(status_code=429, detail="Your Gemini API quota has been exceeded. Please check your plan and billing details at Google AI Studio.")
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {str(e)}")

@router.get("/health", response_model=HealthResponse)
async def health():
    """
    Verify connectivity to Qdrant Cloud and Neon PostgreSQL.
    Useful for health checks and troubleshooting.
    """
    qdrant_ok = await ping_qdrant()
    neon_ok = await ping_neon()
    
    total_count = await get_chunk_count()
    
    is_healthy = qdrant_ok and neon_ok
    
    return HealthResponse(
        status="ok" if is_healthy else "degraded",
        qdrant="connected" if qdrant_ok else "disconnected",
        neon="connected" if neon_ok else "disconnected",
        chunk_count=total_count
    )
