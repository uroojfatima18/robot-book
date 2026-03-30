"""
retriever.py — The bridge between coordinates (vectors) and content (BookChunks).

This module takes a natural language question, transforms it into a vector,
searches Qdrant, and returns a list of matching BookChunks from the database.
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
    if not question.strip():
        return []

    # 1. Turn the question into a high-dimensional vector
    query_vector = await embed_text(question)

    # 2. Search Qdrant for the closest semantic matches
    # This returns ScoredPoint objects which contain the chunk metadata in 'payload'
    scored_points = await search_chunks(vector=query_vector, limit=limit)

    if not scored_points:
        return []

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

    return chunks
