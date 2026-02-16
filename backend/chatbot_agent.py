"""
Complete chatbot implementation using OpenAI API with Gemini-2.0-flash model.
This implementation connects to Google's Gemini API using the OpenAI-compatible endpoint.
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

class GeminiChatbotAgent:
    """
    A chatbot agent that uses the OpenAI API to interact with Google's Gemini API.
    """

    def __init__(self, api_key: Optional[str] = None, model_name: str = "gemini-2.0-flash"):
        """
        Initialize the Gemini chatbot agent.

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

        logger.info(f"Initialized GeminiChatbotAgent with model: {model_name}")

    async def chat(self, message: str, conversation_history: Optional[List[Dict[str, str]]] = None) -> str:
        """
        Send a message to the chatbot and get a response.

        Args:
            message: The user's message
            conversation_history: Optional list of previous messages in the conversation

        Returns:
            The chatbot's response
        """
        try:
            # Prepare the messages for the API call
            messages = []

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
                temperature=0.7,
                max_tokens=1000
            )

            # Extract and return the response
            content = response.choices[0].message.content if response.choices and response.choices[0].message.content else "I couldn't generate a response. Please try again."
            return content

        except Exception as e:
            logger.error(f"Error in chat: {str(e)}")
            raise

    async def stream_chat(self, message: str, conversation_history: Optional[List[Dict[str, str]]] = None) -> AsyncIterator[str]:
        """
        Stream chat responses from the agent (if supported).

        Args:
            message: The user's message
            conversation_history: Optional list of previous messages in the conversation

        Yields:
            Individual response tokens/segments
        """
        try:
            # Prepare the messages for the API call
            messages = []

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
                temperature=0.7,
                max_tokens=1000,
                stream=True
            )

            async for chunk in stream:
                if chunk.choices and chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except Exception as e:
            logger.error(f"Error in stream_chat: {str(e)}")
            raise

    async def close(self):
        """
        Clean up resources when done.
        """
        if hasattr(self, 'client'):
            # Close any connections if needed
            # The AsyncOpenAI client typically handles connection lifecycle automatically
            # We don't need to manually close it in most cases
            pass
        logger.info("Closed GeminiChatbotAgent")


class SimpleChatInterface:
    """
    A simple interface for interacting with the chatbot in a console application.
    """

    def __init__(self, agent: GeminiChatbotAgent):
        self.agent = agent
        self.conversation_history: List[Dict[str, str]] = []

    async def start_conversation(self):
        """
        Start an interactive conversation with the chatbot.
        """
        print("🤖 Gemini Chatbot (gemini-2.0-flash)")
        print("Type 'quit' to exit, 'clear' to reset conversation")
        print("-" * 50)

        while True:
            try:
                user_input = input("\n>You: ").strip()

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
                print(f"\n🤖: {response}")

                # Update conversation history
                self.conversation_history.append({"role": "user", "content": user_input})
                self.conversation_history.append({"role": "assistant", "content": response})

                # Limit history to prevent context window overflow
                if len(self.conversation_history) > 20:  # Keep last 10 exchanges
                    self.conversation_history = self.conversation_history[-20:]

            except KeyboardInterrupt:
                print("\nGoodbye!")
                break
            except Exception as e:
                print(f"❌ Error: {str(e)}")
                logger.error(f"Conversation error: {str(e)}")


async def main():
    """
    Main function to demonstrate the chatbot.
    """
    chatbot = None
    try:
        # Initialize the chatbot agent
        chatbot = GeminiChatbotAgent(model_name="gemini-2.0-flash")

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
    Function to run a simple test of the chatbot functionality.
    """
    async def test():
        chatbot = None
        try:
            # Initialize the chatbot agent
            chatbot = GeminiChatbotAgent(model_name="gemini-2.0-flash")

            # Test a simple message
            response = await chatbot.chat("Hello, what can you help me with?")
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