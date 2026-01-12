import { GoogleGenerativeAI } from "@google/generative-ai";
import { config } from "dotenv";

config();

async function testNewApiKey() {
  console.log("🔍 Testing Gemini API Key Directly\n");
  console.log("=" .repeat(60));

  const apiKey = process.env.GEMINI_API_KEY;

  if (!apiKey) {
    console.log("❌ No GEMINI_API_KEY found in .env file");
    return;
  }

  console.log("✅ API key found in .env");
  console.log(`   Length: ${apiKey.length} characters`);
  console.log(`   Starts with: ${apiKey.substring(0, 10)}...`);
  console.log(`   Ends with: ...${apiKey.substring(apiKey.length - 6)}\n`);

  // Test with gemini-2.0-flash
  console.log("Testing gemini-2.0-flash model...");
  try {
    const genAI = new GoogleGenerativeAI(apiKey);
    const model = genAI.getGenerativeModel({ model: "gemini-2.0-flash" });

    const result = await model.generateContent("Say 'Hello' in one word");
    const response = await result.response;
    const text = response.text();

    console.log("✅ SUCCESS! API key is working!");
    console.log(`   Response: ${text}`);
    console.log("\n🎉 Your chatbot should work now!");
    console.log("   Try opening: http://localhost:3005\n");

  } catch (error: any) {
    console.log("❌ API key test failed");

    if (error.message?.includes('429') || error.message?.includes('quota')) {
      console.log("\n⚠️  QUOTA EXCEEDED");
      console.log("   This API key has already hit its quota limit.");
      console.log("   Solutions:");
      console.log("   1. Wait 1-2 hours for quota to reset");
      console.log("   2. Create a DIFFERENT Google account");
      console.log("   3. Get a new API key from that account");
      console.log("   4. Update backend/.env with the new key\n");

    } else if (error.message?.includes('API key') || error.message?.includes('invalid')) {
      console.log("\n⚠️  INVALID API KEY");
      console.log("   The API key format is incorrect or invalid.");
      console.log("   Please verify you copied the complete key from:");
      console.log("   https://aistudio.google.com/app/apikey\n");

    } else if (error.message?.includes('404') || error.message?.includes('not found')) {
      console.log("\n⚠️  MODEL NOT AVAILABLE");
      console.log("   The gemini-2.0-flash model is not available for this key.");
      console.log("   This might be a regional restriction.\n");

    } else {
      console.log(`\n   Error: ${error.message?.substring(0, 200)}\n`);
    }
  }

  console.log("=" .repeat(60));
}

testNewApiKey().catch(console.error);
