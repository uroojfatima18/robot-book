"""
main.py — The FastAPI application entrypoint.

Configures CORS for the Docusaurus frontend and mounts the API routes.
Initializes database connections and structures on startup.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from app.api.routes import router
from app.db.qdrant_client import ensure_collection
from app.db.neon_client import ensure_table
from app.config import FRONTEND_URL

app = FastAPI(
    title="Physical AI Book RAG Backend",
    description="Minimal RAG chatbot using OpenAI Agents SDK + Gemini + Qdrant + Neon",
    version="1.0.0"
)

# --- Middleware ---

# Allow Docusaurus (GitHub Pages) to communicate with this backend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[FRONTEND_URL],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Events ---/s/


@app.on_event("startup")
async def startup_event():
    """ Runs once when the server starts. """
    print("[Startup] Initializing storage...")
    try:
        # Create Qdrant collection if it doesn't exist
        await ensure_collection()
        # Create Neon Postgres table if it doesn't exist
        await ensure_table()
        print("[Startup] Storage initialization complete.")
    except Exception as e:
        print(f"[Startup] CRITICAL ERROR during initialization: {e}")

# --- Routes ---

app.include_router(router)

@app.get("/")
async def root():
    return {"message": "Physical AI Book RAG API is running. See /docs for usage."}

# --- Runner ---

if __name__ == "__main__":
    # Start the application manually if run as a script
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
