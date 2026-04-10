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

# Health check
@app.get("/api/health")
async def health():
    return {
        "status": "healthy",
        "message": "Backend is running"
    }

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