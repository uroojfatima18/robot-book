"""
qdrant_client.py — Qdrant Cloud operations

Responsibilities:
  - ensure_collection()  : create 'book_chunks' collection if not present
  - upsert_chunks()      : store vectors + payloads (called during ingestion)
  - search_chunks()      : find top-K similar chunks (called during /ask)
"""

from qdrant_client import AsyncQdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    ScoredPoint,
)

from app.config import (
    QDRANT_URL,
    QDRANT_API_KEY,
    COLLECTION_NAME,
    VECTOR_SIZE,
    RETRIEVAL_TOP_K,
)

# Module-level client (initialized once, reused across requests)
_client: AsyncQdrantClient = AsyncQdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)


async def ensure_collection() -> None:
    """
    Create or recreate the 'book_chunks' Qdrant collection.
    If the existing collection has a different vector size (e.g. 1536 vs 768),
    it will be deleted and recreated to match the current embedding model.
    """
    existing = await _client.get_collections()
    existing_names = [c.name for c in existing.collections]

    should_recreate = False
    if COLLECTION_NAME in existing_names:
        # Check existing vector size
        info = await _client.get_collection(collection_name=COLLECTION_NAME)
        current_size = info.config.params.vectors.size
        if current_size != VECTOR_SIZE:
            print(f"[Qdrant] Vector size mismatch ({current_size} != {VECTOR_SIZE}). Recreating...")
            await _client.delete_collection(collection_name=COLLECTION_NAME)
            should_recreate = True
    else:
        should_recreate = True

    if should_recreate:
        await _client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=VECTOR_SIZE,
                distance=Distance.COSINE,
            ),
        )
        print(f"[Qdrant] Created collection '{COLLECTION_NAME}' ({VECTOR_SIZE}-dim, COSINE)")
    else:
        print(f"[Qdrant] Collection '{COLLECTION_NAME}' ready")


async def upsert_chunks(points: list[PointStruct]) -> None:
    """
    Upsert a batch of PointStructs into Qdrant.
    Uses the chunk UUID as the point ID — re-ingestion is idempotent.
    """
    await _client.upsert(
        collection_name=COLLECTION_NAME,
        points=points,
        wait=True,
    )


async def search_chunks(
    vector: list[float],
    limit: int = RETRIEVAL_TOP_K,
) -> list[ScoredPoint]:
    """
    Search Qdrant for the top-K most similar chunks to the given vector.
    Returns scored points with full payload attached.
    Returns an empty list if nothing is found.
    """
    results = await _client.search(
        collection_name=COLLECTION_NAME,
        query_vector=vector,
        limit=limit,
        with_payload=True,
    )
    return results


async def get_vector_count() -> int:
    """Return the total number of vectors in the collection (used by /health)."""
    try:
        info = await _client.get_collection(collection_name=COLLECTION_NAME)
        return info.vectors_count or 0
    except Exception:
        return 0


async def ping_qdrant() -> bool:
    """Return True if Qdrant is reachable, False otherwise (used by /health)."""
    try:
        await _client.get_collections()
        return True
    except Exception:
        return False
