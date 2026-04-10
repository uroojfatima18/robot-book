"""FastAPI RAG Chatbot Backend"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import logging

load_dotenv()

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Robot Book RAG Chatbot",
    description="Retrieval-Augmented Generation chatbot for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Welcome endpoint
@app.get("/")
async def welcome():
    return {
        "message": "RAG Chatbot Backend",
        "docs": "/docs",
        "health": "/api/health",
        "endpoints": {
            "ask": "POST /api/ask (query the book)",
            "health": "GET /api/health (check service status)"
        }
    }

# Startup event to ensure database tables and collections exist
@app.on_event("startup")
async def startup_event():
    logger.info("🚀 Backend is starting up...")
    try:
        from app.db.neon_client import ensure_table
        from app.db.qdrant_client import ensure_collection
        
        logger.info("[DB] Checking Neon PostgreSQL...")
        await ensure_table()
        
        logger.info("[DB] Checking Qdrant Cloud...")
        # ensure_collection is synchronous in our client
        ensure_collection()
        
        logger.info("✅ Startup sequence complete.")
    except Exception as e:
        logger.error(f"❌ Error during startup sequence: {e}")
        # We don't exit, we let the app run so we can debug.

# Included routes are below
# Health and Ask are handled by the router in app/api/routes.py

# Try to include routes with error handling
try:
    from app.api.routes import router
    logger.info("Routes imported successfully")
    app.include_router(router, prefix="/api", tags=["RAG Chatbot"])
except Exception as e:
    logger.error(f"Failed to import routes: {e}")
    # Fallback endpoint
    @app.post("/api/ask")
    async def ask(payload: dict):
        return {
            "answer": "This information is not present in the book.",
            "sources": []
        }