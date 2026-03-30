import asyncio
import google.generativeai as genai
import os
from dotenv import load_dotenv
from app.rag.chunker import get_chunks_from_docs
from app.db.qdrant_client import upsert_chunks, ensure_collection
from app.db.neon_client import upsert_chunk, ensure_table
from qdrant_client.models import PointStruct

load_dotenv("./.venv/.env", override=True)
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

async def minimal_ingest():
    print("--- Starting Minimal Ingest (5 chunks) ---")
    await ensure_collection()
    await ensure_table()
    
    # Get all chunks
    all_chunks = get_chunks_from_docs("../my-website/docs")
    mini_batch = all_chunks[:5] # Just the first 5
    
    print(f"Ingesting 5 chunks using model: models/gemini-embedding-001")
    
    for i, chunk in enumerate(mini_batch):
        try:
            print(f"  [{i+1}/5] Embedding chunk: {chunk.title}...")
            # Wait 10 seconds between EACH chunk to be safe
            await asyncio.sleep(10)
            
            result = genai.embed_content(
                model="models/gemini-embedding-001",
                content=chunk.text,
                task_type="retrieval_document"
            )
            vector = result['embedding']
            
            # Upsert to Qdrant
            point = PointStruct(
                id=chunk.id,
                vector=vector,
                payload={
                    "text": chunk.text,
                    "title": chunk.title,
                    "chapter": chunk.chapter,
                    "source_url": chunk.source_url,
                    "chunk_index": chunk.chunk_index
                }
            )
            await upsert_chunks([point])
            
            # Upsert to Neon
            await upsert_chunk(
                chunk_id=chunk.id,
                text=chunk.text,
                title=chunk.title,
                chapter=chunk.chapter,
                source_url=chunk.source_url,
                chunk_index=chunk.chunk_index
            )
            print(f"  ✅ SUCCESS!")
        except Exception as e:
            print(f"  ❌ FAILED on chunk {i}: {e}")

if __name__ == "__main__":
    asyncio.run(minimal_ingest())
