import asyncio
from qdrant_client import AsyncQdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

async def test_qdrant():
    url = os.getenv("QDRANT_URL")
    if not url.endswith(":6333"):
        url = url.rstrip("/") + ":6333"
    api_key = os.getenv("QDRANT_API_KEY")
    print(f"Testing Qdrant at: {url}")
    client = AsyncQdrantClient(url=url, api_key=api_key)
    try:
        collections = await client.get_collections()
        print(f"Successfully connected! Collections: {collections}")
    except Exception as e:
        print(f"Failed to connect: {e}")

if __name__ == "__main__":
    asyncio.run(test_qdrant())
