#!/usr/bin/env python3
"""Quick test of the embedding pipeline"""

import asyncio
from app.rag.embedder import embed_text

async def test():
    try:
        print("Testing embedding...")
        result = await embed_text('Hello world test')
        print(f'✅ Embedding successful!')
        print(f'   Dimensions: {len(result)}')
        print(f'   First 5 values: {result[:5]}')
    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test())
