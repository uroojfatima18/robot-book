"""
qdrant_client.py — Qdrant Cloud operations

Responsibilities:
  - ensure_collection()  : create 'book_chunks' collection if not present
  - upsert_chunks()      : store vectors + payloads (called during ingestion)
  - search_chunks()      : find top-K similar chunks (called during /ask)
"""

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    ScoredPoint,
)
import logging

from app.config import (
    QDRANT_URL,
    QDRANT_API_KEY,
    COLLECTION_NAME,
    VECTOR_SIZE,
    RETRIEVAL_TOP_K,
)

logger = logging.getLogger(__name__)

# Module-level client (lazy initialization)
_client: QdrantClient = None
_client_initialized = False
_client_error = None


def _get_client() -> QdrantClient:
    """Lazy initialization of Qdrant client with error handling."""
    global _client, _client_initialized, _client_error
    
    if _client_initialized:
        if _client_error:
            raise RuntimeError(f"Qdrant client failed to initialize: {_client_error}")
        return _client
    
    try:
        _client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
        )
        logger.info("✅ Qdrant client initialized successfully")
        _client_initialized = True
        return _client
    except Exception as e:
        logger.error(f"❌ Failed to initialize Qdrant client: {e}")
        _client_error = str(e)
        _client_initialized = True
        raise


def ensure_collection() -> None:
    """
    Create or recreate the 'book_chunks' Qdrant collection.
    If the existing collection has a different vector size (e.g. 1536 vs 768),
    it will be deleted and recreated to match the current embedding model.
    """
    try:
        client = _get_client()
    except Exception as e:
        logger.error(f"Cannot ensure collection: Qdrant not available: {e}")
        return
    
    try:
        existing = client.get_collections()
        existing_names = [c.name for c in existing.collections]

        should_recreate = False
        if COLLECTION_NAME in existing_names:
            # Check existing vector size
            info = client.get_collection(collection_name=COLLECTION_NAME)
            current_size = info.config.params.vectors.size
            if current_size != VECTOR_SIZE:
                logger.info(f"Vector size mismatch ({current_size} != {VECTOR_SIZE}). Recreating...")
                client.delete_collection(collection_name=COLLECTION_NAME)
                should_recreate = True
        else:
            should_recreate = True

        if should_recreate:
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=VECTOR_SIZE,
                    distance=Distance.COSINE,
                ),
            )
            logger.info(f"Created collection '{COLLECTION_NAME}' ({VECTOR_SIZE}-dim, COSINE)")
        else:
            logger.info(f"Collection '{COLLECTION_NAME}' ready")
    except Exception as e:
        logger.error(f"Error ensuring collection: {e}")


def upsert_chunks(points: list[PointStruct]) -> None:
    """
    Upsert a batch of PointStructs into Qdrant.
    Uses the chunk UUID as the point ID — re-ingestion is idempotent.
    """
    try:
        client = _get_client()
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True,
        )
    except Exception as e:
        logger.error(f"Error upserting chunks: {e}")
        raise


def search_chunks(
    vector: list[float],
    limit: int = RETRIEVAL_TOP_K,
) -> list[ScoredPoint]:
    """
    Search Qdrant for the top-K most similar chunks to the given vector.
    Returns scored points with full payload attached.
    Returns an empty list if nothing is found.
    """
    try:
        client = _get_client()
        print(f"[QDRANT] Client type: {type(client)}, has search method: {hasattr(client, 'search')}")
        
        # Call search with proper parameters
        results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=vector,
            limit=limit,
            with_payload=True,
        )
        print(f"[QDRANT] Search returned {len(results)} results")
        return results
    except Exception as e:
        logger.warning(f"Error searching chunks (Qdrant unavailable): {type(e).__name__}: {e}")
        print(f"[QDRANT] Search error: {type(e).__name__}: {e}")
        return []  # Return empty list on error (fallback mode)


def get_vector_count() -> int:
    """Return the total number of vectors in the collection (used by /health)."""
    try:
        client = _get_client()
        info = client.get_collection(collection_name=COLLECTION_NAME)
        return info.vectors_count or 0
    except Exception as e:
        logger.debug(f"Could not get vector count: {e}")
        return 0


def ping_qdrant() -> bool:
    """Return True if Qdrant is reachable, False otherwise (used by /health)."""
    try:
        client = _get_client()
        client.get_collections()
        return True
    except Exception as e:
        logger.debug(f"Qdrant ping failed: {e}")
        return False
