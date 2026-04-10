"""
embedder.py - Generate high-quality vector embeddings for book chunks and search queries.

Uses OpenRouter's OpenAI compatible endpoint for 'text-embedding-3-small' model (1536 dimensions).
All I/O is asynchronous to integrate smoothly with FastAPI.
"""

from openai import AsyncOpenAI
from app.config import OPENROUTER_API_KEY, EMBEDDING_MODEL

# Configure OpenRouter client for embeddings
_embedding_client = AsyncOpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1"
)

async def embed_text(text: str) -> list[float]:
    """
    Generate a 1536-dimensional embedding vector using OpenRouter's text-embedding-3-small.
    Has a 8-second timeout to prevent hanging.
    """
    if not text.strip():
        return [0.0] * 1536

    try:
        import asyncio
        response = await asyncio.wait_for(
            _embedding_client.embeddings.create(
                model=EMBEDDING_MODEL,
                input=text
            ),
            timeout=8  # 8 second timeout
        )
        return response.data[0].embedding
    except asyncio.TimeoutError:
        print(f"[Embedder] OpenRouter request timed out")
        return [0.0] * 1536
    except Exception as e:
        print(f"[Embedder] OpenRouter Embedding Error: {e}")
        return [0.0] * 1536

async def embed_batch(texts: list[str]) -> list[list[float]]:
    """
    Generate embeddings for a list of strings using OpenRouter.
    Has a 15-second timeout to prevent hanging.
    """
    if not texts:
        return []

    try:
        import asyncio
        response = await asyncio.wait_for(
            _embedding_client.embeddings.create(
                model=EMBEDDING_MODEL,
                input=texts
            ),
            timeout=15  # 15 second timeout for batch
        )
        return [item.embedding for item in response.data]
    except asyncio.TimeoutError:
        print(f"[Embedder] OpenRouter batch request timed out")
        return [[0.0] * 1536 for _ in texts]
    except Exception as e:
        print(f"[Embedder] OpenRouter Batch Embedding Error: {e}")
        return [[0.0] * 1536 for _ in texts]
