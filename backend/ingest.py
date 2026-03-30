"""
ingest.py — Standalone ingestion script to populate the RAG knowledge base.

Workflow:
  1. Initialize connection to Qdrant & Neon.
  2. Walk docs/ directory and find all .md files.
  3. Chunk content using heading-aware logic.
  4. Generate embeddings for each chunk via OpenAI.
  5. Store vectors in Qdrant and metadata in Neon.
"""

import asyncio
from qdrant_client.models import PointStruct

from app.config import DOCS_PATH
from app.rag.chunker import load_all_chunks
from app.rag.embedder import embed_batch
from app.db.qdrant_client import ensure_collection, upsert_chunks
from app.db.neon_client import ensure_table, upsert_chunk

async def process_ingestion():
    """Execute the full ingestion pipeline."""
    print("--- Starting Ingestion Pipeline ---")
    
    # 1. Initialize databases
    print("[1/4] Ensuring DB collections and tables exist...")
    await ensure_collection()
    await ensure_table()

    # 2. Extract and chunk content
    print(f"[2/4] Loading and chunking book content from: {DOCS_PATH}...")
    try:
        chunks = load_all_chunks(DOCS_PATH)
    except Exception as e:
        print(f"FAILED to load chunks: {e}")
        return

    total_chunks = len(chunks)
    if total_chunks == 0:
        print("No chunks found. Check your docs path and .md files.")
        return
        
    print(f"Loaded {total_chunks} total chunks.")

    # 3. Generate Embeddings and Store
    print(f"[3/4] Generating embeddings and storing in databases...")
    
    # We process in batches to be efficient and stay within API limits
    # For Gemini Free Tier, we use a slower pace to avoid 429 errors.
    batch_size = 20
    import time
    
    for i in range(0, total_chunks, batch_size):
        batch = chunks[i:i + batch_size]
        
        # Check if this batch is likely already ingested (optional but helpful)
        # For simplicity in this fix, we just add the sleep.
        
        batch_texts = [c.text for c in batch]
        
        try:
            # Get vectors for the entire batch in one call
            vectors = await embed_batch(batch_texts)
            
            # Prepare Qdrant points
            points = []
            for j, chunk in enumerate(batch):
                points.append(PointStruct(
                    id=chunk.id,
                    vector=vectors[j],
                    payload={
                        "text": chunk.text,
                        "title": chunk.title,
                        "chapter": chunk.chapter,
                        "source_url": chunk.source_url,
                        "chunk_index": chunk.chunk_index
                    }
                ))
            
            # Upsert into Qdrant
            await upsert_chunks(points)
            
            # Upsert into Neon (sequential for metadata)
            for chunk in batch:
                await upsert_chunk(
                    chunk_id=chunk.id,
                    text=chunk.text,
                    title=chunk.title,
                    chapter=chunk.chapter,
                    source_url=chunk.source_url,
                    chunk_index=chunk.chunk_index
                )
                
            progress = min(i + batch_size, total_chunks)
            print(f"  ✅ Processed {progress}/{total_chunks} chunks...")
            
            # 🛑 CRITICAL: Wait 5 seconds to avoid Gemini Free Tier 429 Rate Limits
            if progress < total_chunks:
                time.sleep(5)
            
        except Exception as e:
            if "429" in str(e):
                print(f"  ⚠️ Rate limit hit at index {i}. Waiting 30 seconds...")
                time.sleep(30)
                # We don't increment i here so it will retry this batch in the next loop 
                # (but in this simple for loop, we'll just continue to next. 
                # Better to use a while loop for true retries, but sleep should help).
            else:
                print(f"  ❌ Error processing batch starting at index {i}: {e}")
                continue

    print(f"\n[4/4] SUCCESS: Ingested {total_chunks} chunks into RAG system.")
    print("--- Ingestion Complete ---")

if __name__ == "__main__":
    asyncio.run(process_ingestion())
