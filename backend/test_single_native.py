import google.generativeai as genai
import os
from dotenv import load_dotenv

load_dotenv()
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

try:
    print("Testing 1 single embedding...")
    result = genai.embed_content(
        model="models/gemini-embedding-001",
        content="Hello",
        task_type="retrieval_query"
    )
    print("✅ SUCCESS!")
except Exception as e:
    print(f"❌ FAILED: {e}")
