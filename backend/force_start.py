import asyncio
import os
from dotenv import load_dotenv
import google.generativeai as genai
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
import uuid

# 1. Setup
load_dotenv("./.venv/.env", override=True)
api_key = os.getenv("GEMINI_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
qdrant_key = os.getenv("QDRANT_API_KEY")

genai.configure(api_key=api_key)
q_client = AsyncQdrantClient(url=qdrant_url, api_key=qdrant_key)

async def force_work():
    print("🚀 Forcing Chatbot to work...")
    
    # Tiny piece of knowledge from your Intro
    text = "This book, 'Physical AI & Humanoid Robotics', is a comprehensive guide to building intelligent machines. It covers ROS2, actuators, and the nervous system of robots."
    
    try:
        print("Step 1: Embedding 1 single chunk...")
        result = genai.embed_content(
            model="models/gemini-embedding-001",
            content=text,
            task_type="retrieval_document"
        )
        vector = result['embedding']
        
        print("Step 2: Pushing to Qdrant...")
        point_id = str(uuid.uuid4())
        await q_client.upsert(
            collection_name="book_chunks",
            points=[
                PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={"text": text, "title": "Introduction"}
                )
            ]
        )
        print("✅ SUCCESS! Knowledge is in the database.")
        print("You can now ask: 'What is this book about?'")
        
    except Exception as e:
        print(f"❌ Still blocked by Google: {e}")
    finally:
        await q_client.close()

if __name__ == "__main__":
    asyncio.run(force_work())
