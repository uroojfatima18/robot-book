"""
book_agent.py - The reasoning engine powered by OpenRouter.

This module defines the strict "BookOnlyAgent" using the OpenAI Agents SDK.
The agent is instructed to only answer based on the provided context.
"""

from agents import (
    Agent,
    OpenAIChatCompletionsModel,
    RunConfig,
    AsyncOpenAI,
    Runner
)
from app.config import OPENROUTER_API_KEY

# Configure the async client to speak to OpenRouter
_external_client = AsyncOpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1",
)

# Define the model via OpenRouter (using a capable free/cheap model)
_model = OpenAIChatCompletionsModel(
    model="stepfun/step-3.5-flash:free",
    openai_client=_external_client,
)

# Global run configuration
_run_config = RunConfig(
    model=_model,
    model_provider=_external_client,
)

# The core Agent with strict instructions to prevent hallucination
book_agent = Agent(
    name="BookOnlyAgent",
    instructions="""
You are an expert AI assistant for the Physical AI & Humanoid Robotics book.
Your primary goal is to help readers understand the book's content.

STRICT RULES:
1. Answer ONLY using the provided "Book Context" below.
2. If the answer is NOT present in the Book Context, you must respond EXACTLY with:
   "This information is not present in the book."
3. Do NOT use any outside knowledge or provide links that are not in the context.
4. Keep your answers clear, educational, and professional.
""",
    model=_model,
)

async def ask_book(question: str, context: str) -> str:
    """
    Asks the Gemini agent to answer the question based on the provided book context.
    
    This function is only called after relevant chunks are found.
    If no chunks were found, the route handler returns the fallback message directly.
    """
    prompt = f"""
Book Context:
{context}

Question:
{question}
"""

    try:
        result = await Runner.run(
            book_agent,
            input=prompt,
            run_config=_run_config,
        )
        return result.final_output
    except Exception as e:
        print(f"[Agent] Error during LLM call: {e}")
        return "Sorry, I encountered an error while processing your request."
