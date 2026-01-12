import { config } from "dotenv";
import { streamChatResponse, generateTitle } from "./src/lib/chat";

// Load environment variables
config();

async function testChatImplementation() {
  console.log("🧪 Testing OpenAI SDK with Gemini Endpoint\n");

  try {
    // Test 1: Generate Title
    console.log("Test 1: Generating conversation title...");
    const title = await generateTitle("What is ROS 2 and how does it work?");
    console.log(`✅ Title generated: "${title}"\n`);

    // Test 2: Stream Chat Response
    console.log("Test 2: Streaming chat response...");
    const context = "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.";
    const question = "What is ROS 2?";

    let fullResponse = "";
    let chunkCount = 0;

    for await (const chunk of streamChatResponse(context, question)) {
      fullResponse += chunk;
      chunkCount++;
      process.stdout.write(".");
    }

    console.log(`\n✅ Received ${chunkCount} chunks`);
    console.log(`📝 Response preview: ${fullResponse.substring(0, 150)}...\n`);

    console.log("✅ All tests passed!");
    console.log("\n🎉 Migration successful! OpenAI SDK is working with Gemini endpoint.");

  } catch (error: any) {
    console.error("\n❌ Test failed:", error.message);

    if (error.message === "QUOTA_EXCEEDED") {
      console.error("⚠️  Gemini API quota exceeded. Wait a few minutes and try again.");
    } else if (error.message === "INVALID_API_KEY") {
      console.error("⚠️  Invalid API key. Check your GEMINI_API_KEY in .env file.");
    } else {
      console.error("Full error:", error);
    }

    process.exit(1);
  }
}

testChatImplementation();
