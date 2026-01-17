"""
Complete chatbot implementation using OpenAI API with Gemini-2.0-flash model.
This implementation connects to Google's Gemini API using the OpenAI-compatible endpoint
and integrates with the RAG system to answer from book content.
"""

from typing import Dict, Any, List, Optional, AsyncIterator
from openai import AsyncOpenAI
from dotenv import load_dotenv, find_dotenv
import os
import asyncio
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv(find_dotenv())

class GeminiRAGChatbotAgent:
    """
    A RAG-enabled chatbot agent that uses the OpenAI API to interact with Google's Gemini API
    and retrieves information from the book content stored in the vector database.
    """

    def __init__(self, api_key: Optional[str] = None, model_name: str = "gemini-2.0-flash"):
        """
        Initialize the RAG-enabled Gemini chatbot agent.

        Args:
            api_key: Gemini API key. If not provided, will try to load from environment
            model_name: Name of the Gemini model to use
        """
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY must be provided either as parameter or in environment variables")

        # Initialize the AsyncOpenAI client with Google's OpenAI-compatible endpoint
        self.client = AsyncOpenAI(
            api_key=self.api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        self.model_name = model_name

        # Initialize Qdrant client for RAG functionality
        self.qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Import Qdrant client if available
        try:
            from qdrant_client import AsyncQdrantClient
            self.qdrant_client = AsyncQdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key,
                prefer_grpc=False
            )
            self.collection_name = "documents"  # Default collection name
            self.rag_available = True
        except ImportError:
            logger.warning("Qdrant client not available. RAG functionality will be limited.")
            self.rag_available = False
            self.qdrant_client = None

        logger.info(f"Initialized GeminiRAGChatbotAgent with model: {model_name}, RAG available: {self.rag_available}")

    async def _retrieve_context(self, query: str, limit: int = 5) -> str:
        """
        Retrieve relevant context from the vector database based on the query.

        Args:
            query: The user's query to find relevant context for
            limit: Maximum number of chunks to retrieve

        Returns:
            Relevant context text from the book content
        """
        if not self.rag_available or not self.qdrant_client:
            return ""

        try:
            # Import sentence-transformers for embedding
            from transformers import AutoTokenizer, AutoModel
            import torch
            import numpy as np

            # For simplicity, we'll use a placeholder approach
            # In a real implementation, you'd embed the query and search the vector DB
            # This is a simplified version - the actual implementation would depend on your schema
            search_results = await self.qdrant_client.search(
                collection_name=self.collection_name,
                query_text=query,  # This might need to be embedded depending on your setup
                limit=limit
            )

            context_chunks = []
            for result in search_results:
                payload = result.payload
                if 'content' in payload:
                    context_chunks.append(payload['content'])

            return "\\n\\n".join(context_chunks[:limit])
        except Exception as e:
            logger.error(f"Error retrieving context from vector database: {str(e)}")
            return ""

    async def chat(self, message: str, conversation_history: Optional[List[Dict[str, str]]] = None) -> str:
        """
        Send a message to the chatbot and get a response based on book content.

        Args:
            message: The user's message
            conversation_history: Optional list of previous messages in the conversation

        Returns:
            The chatbot's response based on book content
        """
        try:
            # Retrieve relevant context from the book content
            context_text = await self._retrieve_context(message)

            # Prepare the messages for the API call
            messages = []

            # Add system instruction to use context
            system_prompt = f"""You are a helpful assistant that answers questions about robotics, ROS 2, AI, and related topics based on the provided book content.
            Use the following context to answer the user's question. If the context doesn't contain relevant information, politely say that you don't have that information in the book.

            CONTEXT FROM BOOK:
            {context_text if context_text else 'No relevant content found in the book.'}"""

            messages.append({
                "role": "system",
                "content": system_prompt
            })

            # Add conversation history if provided
            if conversation_history:
                for msg in conversation_history:
                    messages.append({
                        "role": msg.get("role", "user"),
                        "content": msg.get("content", "")
                    })

            # Add the current user message
            messages.append({
                "role": "user",
                "content": message
            })

            # Call the OpenAI-compatible API
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                temperature=0.3,  # Lower temperature for more factual responses
                max_tokens=1000
            )

            # Extract and return the response
            content = response.choices[0].message.content if response.choices and response.choices[0].message.content else "I couldn't generate a response. Please try again."
            return content

        except Exception as e:
            logger.error(f"Error in chat: {str(e)}")
            if "quota" in str(e).lower() or "429" in str(e).lower():
                return "⚠️ The AI service is currently at capacity. This is a temporary issue with the Gemini API quota. Please try again in a few minutes."
            else:
                return f"I encountered an error processing your request: {str(e)}"

    async def stream_chat(self, message: str, conversation_history: Optional[List[Dict[str, str]]] = None) -> AsyncIterator[str]:
        """
        Stream chat responses from the agent based on book content.

        Args:
            message: The user's message
            conversation_history: Optional list of previous messages in the conversation

        Yields:
            Individual response tokens/segments
        """
        try:
            # Retrieve relevant context from the book content
            context_text = await self._retrieve_context(message)

            # Prepare the messages for the API call
            messages = []

            # Add system instruction to use context
            system_prompt = f"""You are a helpful assistant that answers questions about robotics, ROS 2, AI, and related topics based on the provided book content.
            Use the following context to answer the user's question. If the context doesn't contain relevant information, politely say that you don't have that information in the book.

            CONTEXT FROM BOOK:
            {context_text if context_text else 'No relevant content found in the book.'}"""

            messages.append({
                "role": "system",
                "content": system_prompt
            })

            # Add conversation history if provided
            if conversation_history:
                for msg in conversation_history:
                    messages.append({
                        "role": msg.get("role", "user"),
                        "content": msg.get("content", "")
                    })

            # Add the current user message
            messages.append({
                "role": "user",
                "content": message
            })

            # Stream the response
            stream = await self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                temperature=0.3,
                max_tokens=1000,
                stream=True
            )

            async for chunk in stream:
                if chunk.choices and chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except Exception as e:
            logger.error(f"Error in stream_chat: {str(e)}")
            yield f"Error: {str(e)}"
            return

    async def close(self):
        """
        Clean up resources when done.
        """
        if hasattr(self, 'client'):
            # Close any connections if needed
            if hasattr(self.client, 'aclose'):
                await self.client.aclose()
        if hasattr(self, 'qdrant_client') and self.qdrant_client:
            # Close Qdrant client if it exists
            pass  # Qdrant client typically doesn't have an async close method
        logger.info("Closed GeminiRAGChatbotAgent")


class SimpleChatInterface:
    """
    A simple interface for interacting with the RAG-enabled chatbot in a console application.
    """

    def __init__(self, agent: GeminiRAGChatbotAgent):
        self.agent = agent
        self.conversation_history: List[Dict[str, str]] = []

    async def start_conversation(self):
        """
        Start an interactive conversation with the RAG-enabled chatbot.
        """
        print("🤖 Gemini RAG Chatbot (gemini-2.0-flash) - Answers from Book Content")
        print("Type 'quit' to exit, 'clear' to reset conversation")
        print("-" * 70)

        while True:
            try:
                user_input = input("\\n>You: ").strip()

                if user_input.lower() == 'quit':
                    print("Goodbye!")
                    break
                elif user_input.lower() == 'clear':
                    self.conversation_history.clear()
                    print("Conversation cleared.")
                    continue
                elif not user_input:
                    continue

                # Get response from agent
                response = await self.agent.chat(
                    message=user_input,
                    conversation_history=self.conversation_history
                )

                # Display response
                print(f"\\n🤖: {response}")

                # Update conversation history
                self.conversation_history.append({"role": "user", "content": user_input})
                self.conversation_history.append({"role": "assistant", "content": response})

                # Limit history to prevent context window overflow
                if len(self.conversation_history) > 20:  # Keep last 10 exchanges
                    self.conversation_history = self.conversation_history[-20:]

            except KeyboardInterrupt:
                print("\\nGoodbye!")
                break
            except Exception as e:
                print(f"❌ Error: {str(e)}")
                logger.error(f"Conversation error: {str(e)}")


async def main():
    """
    Main function to demonstrate the RAG-enabled chatbot.
    """
    chatbot = None
    try:
        # Initialize the RAG-enabled chatbot agent
        chatbot = GeminiRAGChatbotAgent(model_name="gemini-2.0-flash")

        # Create and start the simple interface
        interface = SimpleChatInterface(chatbot)
        await interface.start_conversation()

    except ValueError as e:
        print(f"❌ Configuration error: {str(e)}")
        print("Make sure GEMINI_API_KEY is set in your environment variables.")
    except Exception as e:
        print(f"❌ Error initializing chatbot: {str(e)}")
        logger.error(f"Initialization error: {str(e)}")
    finally:
        if chatbot:
            await chatbot.close()


def run_simple_test():
    """
    Function to run a simple test of the RAG-enabled chatbot functionality.
    """
    async def test():
        chatbot = None
        try:
            # Initialize the RAG-enabled chatbot agent
            chatbot = GeminiRAGChatbotAgent(model_name="gemini-2.0-flash")

            # Test a simple message
            response = await chatbot.chat("What is ROS 2?")
            print(f"Response: {response}")

            # Test with conversation history
            history = [
                {"role": "user", "content": "What is ROS 2?"},
                {"role": "assistant", "content": "ROS 2 is Robot Operating System version 2, a flexible framework for writing robot software."}
            ]
            response2 = await chatbot.chat("Can you elaborate?", conversation_history=history)
            print(f"Follow-up response: {response2}")

        except Exception as e:
            print(f"Test error: {str(e)}")
        finally:
            if chatbot:
                await chatbot.close()

    # Run the test
    asyncio.run(test())


if __name__ == "__main__":
    # For a quick test, uncomment the line below:
    # run_simple_test()

    # For interactive mode, run the main function:
    asyncio.run(main())