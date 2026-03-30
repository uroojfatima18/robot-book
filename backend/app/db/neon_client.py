"""
neon_client.py — Neon Serverless PostgreSQL operations

Responsibilities:
  - ensure_table()    : create 'chunks' table if not present
  - upsert_chunk()    : insert or update a single chunk row
  - get_chunk_count() : count stored chunks (used by /health)
  - ping_neon()       : verify connectivity (used by /health)

Uses asyncpg with a lazy connection pool (created on first use).
"""

import asyncpg
from asyncpg import Pool

from app.config import DATABASE_URL

# Lazy pool: created on first call to get_pool()
_pool: Pool | None = None


async def get_pool() -> Pool:
    """Return (or create) the shared asyncpg connection pool."""
    global _pool
    if _pool is None:
        _pool = await asyncpg.create_pool(
            dsn=DATABASE_URL,
            min_size=1,
            max_size=5,
            command_timeout=30,
        )
    return _pool


async def ensure_table() -> None:
    """
    Create the 'chunks' table in Neon PostgreSQL if it does not exist.
    Called once at FastAPI startup and at the start of ingestion.
    """
    pool = await get_pool()
    async with pool.acquire() as conn:
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chunks (
                id           TEXT         PRIMARY KEY,
                text         TEXT         NOT NULL,
                title        VARCHAR(500),
                chapter      VARCHAR(200),
                source_url   VARCHAR(500),
                chunk_index  INTEGER      NOT NULL DEFAULT 0,
                created_at   TIMESTAMPTZ  DEFAULT NOW()
            )
        """)
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_chunks_chapter
            ON chunks(chapter)
        """)
    print("[Neon] 'chunks' table ready")


async def upsert_chunk(chunk_id: str, text: str, title: str,
                       chapter: str, source_url: str, chunk_index: int) -> None:
    """
    Insert a chunk row, or update it if the UUID already exists.
    This makes ingestion idempotent — safe to re-run without duplicating data.
    """
    pool = await get_pool()
    async with pool.acquire() as conn:
        await conn.execute("""
            INSERT INTO chunks (id, text, title, chapter, source_url, chunk_index)
            VALUES ($1, $2, $3, $4, $5, $6)
            ON CONFLICT (id) DO UPDATE SET
                text        = EXCLUDED.text,
                title       = EXCLUDED.title,
                chapter     = EXCLUDED.chapter,
                source_url  = EXCLUDED.source_url,
                chunk_index = EXCLUDED.chunk_index
        """, chunk_id, text, title, chapter, source_url, chunk_index)


async def get_chunk_count() -> int:
    """Return the number of rows in the 'chunks' table (used by /health)."""
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            row = await conn.fetchrow("SELECT COUNT(*) AS n FROM chunks")
            return row["n"]
    except Exception:
        return 0


async def ping_neon() -> bool:
    """Return True if Neon is reachable, False otherwise (used by /health)."""
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            await conn.fetchval("SELECT 1")
        return True
    except Exception:
        return False
