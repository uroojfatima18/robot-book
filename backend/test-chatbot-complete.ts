import { GoogleGenerativeAI } from "@google/generative-ai";
import { config } from "dotenv";

config();

async function testChatbot() {
  console.log("🔍 CHATBOT DIAGNOSTIC TEST\n");
  console.log("=" .repeat(60));

  // 1. Test Environment Variables
  console.log("\n1️⃣ ENVIRONMENT VARIABLES:");
  console.log("   GEMINI_API_KEY:", process.env.GEMINI_API_KEY ? "✅ Set" : "❌ Missing");
  console.log("   QDRANT_URL:", process.env.QDRANT_URL ? "✅ Set" : "❌ Missing");
  console.log("   QDRANT_API_KEY:", process.env.QDRANT_API_KEY ? "✅ Set" : "❌ Missing");

  // 2. Test Gemini API
  console.log("\n2️⃣ GEMINI API TEST:");
  try {
    const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);
    const model = genAI.getGenerativeModel({ model: "gemini-2.0-flash" });

    const result = await model.generateContent("Say 'Hello' in one word");
    const response = await result.response;
    const text = response.text();

    console.log("   ✅ Gemini API is working!");
    console.log("   Response:", text.substring(0, 50));
  } catch (error: any) {
    console.log("   ❌ Gemini API Error:", error.message);
    if (error.message?.includes('429') || error.message?.includes('quota')) {
      console.log("   ⚠️  QUOTA EXCEEDED - Need to wait or get new API key");
    } else if (error.message?.includes('API key')) {
      console.log("   ⚠️  INVALID API KEY - Need to update GEMINI_API_KEY");
    }
  }

  // 3. Test Qdrant Connection
  console.log("\n3️⃣ QDRANT CONNECTION TEST:");
  try {
    const { qdrant, getCollectionStats } = await import("./src/lib/qdrant");
    const stats = await getCollectionStats();
    console.log("   ✅ Qdrant is connected!");
    console.log("   Points in database:", stats.pointsCount);
    console.log("   Vectors count:", stats.vectorsCount);
  } catch (error: any) {
    console.log("   ❌ Qdrant Error:", error.message);
  }

  // 4. Test RAG Retrieval
  console.log("\n4️⃣ RAG RETRIEVAL TEST:");
  try {
    const { retrieveContext } = await import("./src/lib/rag/retrieve");
    const result = await retrieveContext("What is ROS 2?");
    console.log("   ✅ RAG retrieval is working!");
    console.log("   Chunks found:", result.chunks.length);
    console.log("   Total tokens:", result.totalTokens);
    if (result.chunks.length > 0) {
      console.log("   Top match chapter:", result.chunks[0].payload.chapter);
      console.log("   Top match score:", result.chunks[0].score.toFixed(3));
    }
  } catch (error: any) {
    console.log("   ❌ RAG Error:", error.message);
  }

  // 5. Backend Server Status
  console.log("\n5️⃣ BACKEND SERVER STATUS:");
  try {
    const response = await fetch("http://localhost:3001/api/health");
    console.log("   ✅ Backend server is running on port 3001");
  } catch (error) {
    console.log("   ❌ Backend server is not responding");
  }

  // 6. Frontend Server Status
  console.log("\n6️⃣ FRONTEND SERVER STATUS:");
  try {
    const response = await fetch("http://localhost:3005");
    console.log("   ✅ Frontend server is running on port 3005");
  } catch (error) {
    console.log("   ❌ Frontend server is not responding");
  }

  console.log("\n" + "=".repeat(60));
  console.log("\n📋 SUMMARY:");
  console.log("   • Backend API: http://localhost:3001");
  console.log("   • Frontend UI: http://localhost:3005");
  console.log("   • Chatbot endpoint: http://localhost:3001/api/chat/public");
  console.log("\n");
}

testChatbot().catch(console.error);
