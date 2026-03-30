# from fastapi import FastAPI
# from dotenv import load_dotenv
# import os

# from agents import (
#     Agent,
#     OpenAIChatCompletionsModel,
#     AsyncOpenAI,
#     Runner,
#     RunConfig,
# )

# load_dotenv()

# app = FastAPI()

# from fastapi.middleware.cors import CORSMiddleware
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],  # Adjust this for production
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

# # Gemini setup
# client = AsyncOpenAI(
#     api_key=os.getenv("OPENROUTER_API_KEY"),
#     base_url="https://openrouter.ai/api/v1"
# )
# model = OpenAIChatCompletionsModel(
#     model="stepfun/step-3.5-flash:free",
#     openai_client=client,
# )

# run_config = RunConfig(
#     model=model,
#     model_provider=client,
# )

# # Agent
# agent = Agent(
#     name="BookAgent",
#     instructions="""
# You answer ONLY from the provided book text.
# If answer is not present say:
# "This information is not present in the book."
# """,
#     model=model,
# )

# # Temporary book content (hardcoded)
# BOOK_TEXT = """
# This book explains Python basics.
# Python is a high-level programming language.
# FastAPI is used to build APIs in Python.
# """

# @app.post("/ask")
# async def ask(payload: dict):
#     question = payload["question"]

#     prompt = f"""
# Book Content:
# {BOOK_TEXT}

# Question:
# {question}
# """

#     result = await Runner.run(
#         agent,
#         input=prompt,
#         run_config=run_config,
#     )

#     return {
#         "question": question,
#         "answer": result.final_output
#     }



from fastapi import FastAPI
from dotenv import load_dotenv
from fastapi.middleware.cors import CORSMiddleware
from app.api.routes import router

load_dotenv()

app = FastAPI(
    title="Robot Book RAG Chatbot",
    description="Retrieval-Augmented Generation chatbot for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the RAG API routes
app.include_router(router, prefix="/api", tags=["RAG Chatbot"])

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