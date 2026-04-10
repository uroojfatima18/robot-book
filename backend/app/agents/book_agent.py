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
    model="nvidia/nemotron-3-super-120b-a12b:free",
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
1. Answer technical questions ONLY using the provided "Book Context" below.
2. IMPORTANT: Each piece of context has a chapter title. You MUST mention the chapter name in your response (e.g., "According to Chapter 1...", or "You can find more in the section on...").
3. If the answer to a TECHNICAL question is NOT present in the context, respond with: "This information is not present in the book."
4. You are ENCOURAGED to be polite and greet the user freely.
5. Do NOT use outside knowledge for technical facts.
6. Keep your answers clear, educational, and professional.
""",
    model=_model,
)

async def ask_book(question: str, context: str) -> str:
    """
    Asks the Gemini agent to answer the question based on the provided book context.
    
    This function is only called after relevant chunks are found.
    If no chunks were found, the route handler returns the fallback message directly.
    """
    print(f"[AGENT] Called with question: {question}")
    print(f"[AGENT] Context length: {len(context)} chars")
    print(f"[AGENT] Context preview: {context[:200]}...")
    
    prompt = f"""
Book Context:
{context}

Question:
{question}
"""

    try:
        print(f"[AGENT] Invoking LLM with prompt length: {len(prompt)}")
        result = await Runner.run(
            book_agent,
            input=prompt,
            run_config=_run_config,
        )
        print(f"[AGENT] LLM returned: {result.final_output[:100]}")
        return result.final_output
    except Exception as e:
        print(f"[AGENT] Error during LLM call: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return "Sorry, I encountered an error while processing your request."
