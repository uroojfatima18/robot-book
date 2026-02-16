import { config } from "dotenv";
import OpenAI from "openai";

// Load environment variables
config();

async function testGeminiStreaming() {
  console.log("🧪 Testing Gemini OpenAI-compatible endpoint streaming\n");

  const openai = new OpenAI({
    apiKey: process.env.GEMINI_API_KEY!,
    baseURL: "https://generativelanguage.googleapis.com/v1beta/openai/",
  });

  console.log("✅ OpenAI client created");
  console.log(`📍 Base URL: https://generativelanguage.googleapis.com/v1beta/openai/`);
  console.log(`🔑 API Key: ${process.env.GEMINI_API_KEY?.substring(0, 10)}...`);
  console.log();

  try {
    console.log("🚀 Attempting to create streaming completion...");

    const stream = await openai.chat.completions.create({
      model: "gemini-2.0-flash",
      messages: [
        { role: "system", content: "You are a helpful assistant." },
        { role: "user", content: "Say 'Hello World' in exactly 2 words." }
      ],
      stream: true,
      temperature: 0.7,
      max_tokens: 50,
    });

    console.log("✅ Stream created successfully");
    console.log("📥 Receiving chunks...\n");

    let chunkCount = 0;
    let fullResponse = "";

    for await (const chunk of stream) {
      chunkCount++;
      const content = chunk.choices[0]?.delta?.content;

      if (content) {
        fullResponse += content;
        process.stdout.write(content);
      }

      // Log first few chunks for debugging
      if (chunkCount <= 3) {
        console.log(`\n[Chunk ${chunkCount}]:`, JSON.stringify(chunk, null, 2));
      }
    }

    console.log(`\n\n✅ Streaming completed!`);
    console.log(`📊 Total chunks: ${chunkCount}`);
    console.log(`📝 Full response: "${fullResponse}"`);

  } catch (error: any) {
    console.error("\n❌ Error occurred:");
    console.error("Type:", error.constructor.name);
    console.error("Status:", error.status);
    console.error("Message:", error.message);
    console.error("Code:", error.code);

    if (error.response) {
      console.error("Response:", error.response);
    }

    console.error("\nFull error:", error);
    process.exit(1);
  }
}

testGeminiStreaming();
