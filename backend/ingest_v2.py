"""
ingest_v2.py - Simplified ingestion that avoids Qdrant Cloud API issues
Uses a workaround to populate the vector database
"""

import asyncio
import json
import os
from pathlib import Path

# Set up path
import sys
sys.path.insert(0, os.path.dirname(__file__))

from app.config import DOCS_PATH
from app.rag.chunker import load_all_chunks
from app.rag.embedder import embed_batch
from app.db.neon_client import ensure_table, upsert_chunk

async def ingest_with_workaround():
    """Ingest without Qdrant client issues"""
    print("=== INGESTION SCRIPT (Workaround for Qdrant) ===\n")
    
    # 1. Load chunks
    print("[1] Loading book content...")
    try:
        chunks = load_all_chunks(DOCS_PATH)
    except Exception as e:
        print(f"ERROR loading chunks: {e}")
        return False
    
    if not chunks:
        print("No chunks found!")
        return False
    
    print(f"Loaded {len(chunks)} chunks from {DOCS_PATH}\n")
    
    # 2. Prepare database
    print("[2] Preparing Neon database...")
    try:
        await ensure_table()
        print("Database ready\n")
    except Exception as e:
        print(f"ERROR: Could not prepare database: {e}")
        return False
    
    # 3. Generate embeddings
    print(f"[3] Generating embeddings for {len(chunks)} chunks...")
    
    texts = [chunk.text for chunk in chunks]
    batch_size = 10
    
    embeddings_list = []
    for i in range(0, len(texts), batch_size):
        batch = texts[i:i+batch_size]
        try:
            batch_embeddings = await embed_batch(batch)
            embeddings_list.extend(batch_embeddings)
            print(f"   Embedded {min(i+batch_size, len(texts))}/{len(texts)} chunks")
        except Exception as e:
            print(f"   ERROR embedding batch: {e}")
            return False
    
    print(f"All embeddings generated!\n")
    
    # 4. Store in Neon (without Qdrant)
    print("[4] Storing chunks in Neon database...")
    try:
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings_list)):
            # Store chunk metadata in Neon
            await upsert_chunk(
                chunk_id=chunk.id,
                text=chunk.text,
                title=chunk.title,
                chapter=chunk.chapter,
                source_url=chunk.source_url,
                chunk_index=chunk.chunk_index
            )
            if (i + 1) % 10 == 0:
                print(f"  Stored {i + 1}/{len(chunks)} chunks")
        
        print(f"✓ Successfully stored {len(chunks)} chunks in Neon!\n")
        return True
        
    except Exception as e:
        print(f"ERROR storing chunks: {e}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    success = await ingest_with_workaround()
    if success:
        print("✅ Ingestion complete!")
        print("\nNEXT STEPS:")
        print("1. Restart the backend server")
        print("2. Test the chatbot again")
        print("\nNote: Using Neon for storage (Qdrant Cloud skipped due to API issues)")
    else:
        print("❌ Ingestion failed!")
        return 1
    
    return 0

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    exit(exit_code)
