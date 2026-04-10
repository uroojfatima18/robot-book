"""
retriever_neon.py - Simple keyword-based retrieval from Neon database
Fallback when Qdrant is unavailable
"""

import asyncpg
from app.rag.chunker import BookChunk
from app.config import DATABASE_URL

async def retrieve_from_neon(question: str, limit: int = 5) -> list[BookChunk]:
    """
    Simple keyword search in Neon for book chunks.
    Falls back when Qdrant vector search is unavailable.
    """
    print(f"[NEON] Starting search for: {question}")
    
    if not question.strip():
        print(f"[NEON] Question is empty")
        return []
    
    keywords = question.lower().split()
    print(f"[NEON] Split keywords: {keywords}")
    
    try:
        conn = await asyncpg.connect(DATABASE_URL)
        print(f"[NEON] Connected to database")
        
        # Build SQL query - search for keywords in text and title
        where_clauses = []
        params = []
        
        for keyword in keywords:
            if len(keyword) > 2:  # Skip single/double letter words
                params.append(f"%{keyword}%")
                # Build clause without f-string for placeholders
                param_num = len(params)
                where_clauses.append(f"(text ILIKE ${param_num} OR title ILIKE ${param_num})")
        
        print(f"[NEON] Filtered keywords (>2 chars): {[p.strip('%') for p in params]}")
        
        if not where_clauses:
            # No meaningful keywords - just fetch first N chunks
            print(f"[NEON] No meaningful keywords, fetching first {limit} chunks")
            rows = await conn.fetch(
                "SELECT id, text, title, chapter, source_url, chunk_index FROM chunks LIMIT $1",
                limit
            )
        else:
            # Build WHERE clause
            where_sql = " OR ".join(where_clauses)
            # Add limit parameter
            limit_param_num = len(params) + 1
            query = f"SELECT id, text, title, chapter, source_url, chunk_index FROM chunks WHERE {where_sql} ORDER BY length(text) DESC LIMIT ${limit_param_num}"
            
            print(f"[NEON] Running query with {len(params)} search params + limit")
            print(f"[NEON] Query: {query}")
            print(f"[NEON] Params: {params} + [{limit}]")
            
            rows = await conn.fetch(query, *params, limit)
        
        print(f"[NEON] Query returned {len(rows)} rows")
        
        await conn.close()
        
        chunks = []
        for row in rows:
            chunks.append(BookChunk(
                id=row['id'],
                text=row['text'],
                title=row['title'],
                chapter=row['chapter'],
                source_url=row['source_url'],
                chunk_index=row['chunk_index']
            ))
        
        print(f"[NEON] Returning {len(chunks)} chunks")
        return chunks
        
    except Exception as e:
        print(f"[NEON] Error searching Neon: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return []
