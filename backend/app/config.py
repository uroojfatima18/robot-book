"""
config.py — Load and validate all environment variables from .env
All other modules import from here. Never read os.getenv() elsewhere.
"""

import os
from dotenv import load_dotenv, find_dotenv

load_dotenv(find_dotenv())


def _require(key: str) -> str:
    """Get env var or raise clear error on missing value."""
    value = os.getenv(key)
    if not value:
        raise ValueError(f"Missing required environment variable: {key}")
    return value


# LLM (OpenRouter)
OPENROUTER_API_KEY: str = _require("OPENROUTER_API_KEY")

# Embeddings model via OpenRouter
EMBEDDING_MODEL: str = os.getenv("EMBEDDING_MODEL", "openai/text-embedding-3-small")

# Qdrant Cloud
QDRANT_URL: str = _require("QDRANT_URL")
QDRANT_API_KEY: str = _require("QDRANT_API_KEY")

# Neon PostgreSQL
DATABASE_URL: str = _require("DATABASE_URL")

# Book docs path (relative to backend/ directory)
DOCS_PATH: str = os.getenv("DOCS_PATH", "../my-website/docs")

# Frontend origin for CORS
FRONTEND_URL: str = os.getenv("FRONTEND_URL", "*")

# Qdrant collection settings
COLLECTION_NAME: str = "book_chunks"
VECTOR_SIZE: int = 1536  # OpenRouter text-embedding-3-small produces 1536-dimensional vectors

# RAG settings
RETRIEVAL_TOP_K: int = 5
CHUNK_MIN_WORDS: int = 20
CHUNK_MAX_CHARS: int = 2000
