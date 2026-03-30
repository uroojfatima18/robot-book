"""
chunker.py — Read all book Markdown files and split them into BookChunks.

Pipeline:
  load_all_chunks(docs_path)
    ├── walk docs_path recursively for *.md files
    ├── for each file: strip frontmatter → split on headings
    └── return list[BookChunk]

Design decisions:
  - UUID is deterministic (uuid5) so re-ingestion is idempotent
  - Chunks < CHUNK_MIN_WORDS words are skipped (too sparse for RAG)
  - Each chunk is capped at CHUNK_MAX_CHARS characters to stay within embedding limits
  - Chapter is derived from the top-level folder name under docs_path
"""

import os
import re
from pathlib import Path
from uuid import uuid5, NAMESPACE_URL
from dataclasses import dataclass

from app.config import CHUNK_MIN_WORDS, CHUNK_MAX_CHARS


@dataclass
class BookChunk:
    id: str           # deterministic UUID string
    text: str         # chunk body text (heading + content)
    title: str        # extracted heading title
    chapter: str      # top-level folder name e.g. "01-ros2-nervous-system"
    source_url: str   # relative path from docs root e.g. "01-ros2-nervous-system/intro.md"
    chunk_index: int  # position within the source file


def strip_frontmatter(content: str) -> str:
    """
    Remove YAML --- blocks from the top of a Markdown file.

    Example input:
        ---
        id: chapter_1_introduction
        title: "..."
        ---
        # Real content starts here

    Returns content after the closing ---.
    """
    stripped = content.strip()
    if stripped.startswith("---"):
        # Find the closing ---
        end = stripped.find("---", 3)
        if end != -1:
            return stripped[end + 3:].strip()
    return stripped


def extract_chapter(file_path: str, docs_path: str) -> str:
    """
    Extract the chapter name from the file's top-level folder.

    Example:
        docs_path = "../my-website/docs"
        file_path = "../my-website/docs/01-ros2-nervous-system/beginner/01-intro.md"
        → returns "01-ros2-nervous-system"
    """
    try:
        rel = os.path.relpath(file_path, docs_path)
        parts = Path(rel).parts
        if len(parts) > 1:
            return parts[0]
        return "root"
    except Exception:
        return "unknown"


def chunk_markdown(content: str, source_url: str, chapter: str) -> list[BookChunk]:
    """
    Split a single Markdown file's content into heading-aware chunks.

    Strategy:
      - Strip frontmatter first
      - Split on lines that start a new H1 or H2 heading
      - Each chunk = heading + all content until next heading
      - Skip chunks that are too short (< CHUNK_MIN_WORDS words)
      - Cap chunk text at CHUNK_MAX_CHARS characters

    Returns a list of BookChunk objects.
    """
    content = strip_frontmatter(content)

    if not content.strip():
        return []

    # Split at H1 or H2 heading boundaries (keep the heading in each chunk)
    # Regex: split at a newline followed immediately by # or ##
    sections = re.split(r'\n(?=#{1,2} )', content)

    chunks: list[BookChunk] = []

    for i, section in enumerate(sections):
        text = section.strip()

        # Skip empty or trivially short sections
        if len(text.split()) < CHUNK_MIN_WORDS:
            continue

        # Cap to embedding-safe size
        text = text[:CHUNK_MAX_CHARS]

        # Extract title from the first line if it's a heading
        lines = text.split("\n")
        first_line = lines[0].strip()
        if first_line.startswith("#"):
            title = first_line.lstrip("#").strip()
        else:
            # No heading — use source filename as fallback title
            title = Path(source_url).stem.replace("-", " ").title()

        # Deterministic UUID: same file + same index = same UUID on re-ingest
        chunk_id = str(uuid5(NAMESPACE_URL, f"{source_url}#{i}"))

        chunks.append(BookChunk(
            id=chunk_id,
            text=text,
            title=title,
            chapter=chapter,
            source_url=source_url,
            chunk_index=i,
        ))

    return chunks


def load_all_chunks(docs_path: str) -> list[BookChunk]:
    """
    Walk the entire docs directory recursively, load all .md files,
    chunk each one, and return the full list of BookChunks.

    Skips files that are empty or produce no valid chunks.
    Prints progress for visibility during ingestion.
    """
    docs_root = Path(docs_path).resolve()

    if not docs_root.exists():
        raise FileNotFoundError(
            f"Docs path not found: {docs_root}\n"
            f"Check DOCS_PATH in your .env file."
        )

    md_files = sorted(docs_root.rglob("*.md"))

    if not md_files:
        raise RuntimeError(f"No .md files found under {docs_root}")

    all_chunks: list[BookChunk] = []
    print(f"[Chunker] Found {len(md_files)} Markdown files in {docs_root}")

    for md_file in md_files:
        try:
            content = md_file.read_text(encoding="utf-8")
        except Exception as e:
            print(f"  [SKIP] Could not read {md_file.name}: {e}")
            continue

        # Build relative source_url from docs_root
        source_url = str(md_file.relative_to(docs_root)).replace("\\", "/")
        chapter = extract_chapter(str(md_file), str(docs_root))

        file_chunks = chunk_markdown(content, source_url, chapter)

        if not file_chunks:
            print(f"  [SKIP] No valid chunks from: {source_url}")
            continue

        all_chunks.extend(file_chunks)

    print(f"[Chunker] Total chunks ready for ingestion: {len(all_chunks)}")
    return all_chunks
