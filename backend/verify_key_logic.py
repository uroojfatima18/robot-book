import os
import google.generativeai as genai
from dotenv import load_dotenv

# Force reload from the specific .env file
load_dotenv("./.venv/.env", override=True)

key = os.getenv("GEMINI_API_KEY")
print(f"DEBUG: System is currently using key ending in: ...{key[-4:]}")

genai.configure(api_key=key)

try:
    print("Testing one tiny request...")
    # List models to see if we have ANY access
    models = genai.list_models()
    print("✅ CONNECTION SUCCESS! Your key is working.")
except Exception as e:
    print(f"❌ GOOGLE REJECTED IT: {e}")
