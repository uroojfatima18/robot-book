"""
retriever.py — The bridge between coordinates (vectors) and content (BookChunks).

This module takes a natural language question, transforms it into a vector,
searches Qdrant, and returns a list of matching BookChunks from the database.

Falls back to simple keyword search in Neon if Qdrant is unavailable.
"""

from app.rag.embedder import embed_text
from app.db.qdrant_client import search_chunks
from app.rag.chunker import BookChunk

async def retrieve_relevant_chunks(question: str, limit: int = 5) -> list[BookChunk]:
    """
    Search the vector library for chunks relevant to the user's question.
    
    Args:
        question (str): The user's input question.
        limit (int): How many chunks to retrieve (Top-K). Defaults to 5.
        
    Returns:
        list[BookChunk]: List of relevant chunks found in the book.
            Returns an empty list if no chunks are found.
    """
    print(f"\n[RETRIEVER] Starting search for: {question}")
    
    if not question.strip():
        print(f"[RETRIEVER] Question is empty")
        return []

    try:
        # 1. Try to turn the question into a vector using Qdrant
        print(f"[RETRIEVER] Embedding question...")
        query_vector = await embed_text(question)
        print(f"[RETRIEVER] Got vector with {len(query_vector)} dimensions")

        # 2. Search Qdrant for the closest semantic matches
        print(f"[RETRIEVER] Searching Qdrant...")
        scored_points = search_chunks(vector=query_vector, limit=limit)
        print(f"[RETRIEVER] Qdrant returned {len(scored_points)} results")

        if scored_points:
            # 3. Transform the raw search results into structured BookChunk objects
            chunks: list[BookChunk] = []
            for point in scored_points:
                payload = point.payload
                if not payload:
                    continue
                    
                chunks.append(BookChunk(
                    id=str(point.id),
                    text=payload.get("text", ""),
                    title=payload.get("title", "Untitled"),
                    chapter=payload.get("chapter", "unknown"),
                    source_url=payload.get("source_url", ""),
                    chunk_index=payload.get("chunk_index", 0)
                ))
            
            if chunks:
                print(f"[RETRIEVER] Found {len(chunks)} Qdrant chunks. Sample Source URL: {chunks[0].source_url if chunks else 'None'}")
                return chunks
            else:
                print(f"[RETRIEVER] Qdrant returned points but no valid payloads")
            
    except Exception as e:
        print(f"[RETRIEVER] Qdrant search failed: {type(e).__name__}: {e}")
    
    # Fallback to Neon keyword search if Qdrant fails
    print(f"[RETRIEVER] Falling back to Neon keyword search...")
    try:
        from app.rag.retriever_neon import retrieve_from_neon
        neon_chunks = await retrieve_from_neon(question, limit)
        print(f"[RETRIEVER] Neon search returned {len(neon_chunks)} chunks")
        return neon_chunks
    except Exception as e:
        print(f"[RETRIEVER] Neon search also failed: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return []
