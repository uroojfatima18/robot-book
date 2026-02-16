import { GoogleGenerativeAI } from "@google/generative-ai";
import { config } from "dotenv";

config();

const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY || "");

const modelsToTry = [
  "gemini-1.5-pro",
  "gemini-1.5-flash-001",
  "gemini-1.5-flash-002",
  "gemini-pro",
  "gemini-1.0-pro",
];

async function testModels() {
  console.log("🔍 Testing Gemini Models...\n");

  for (const modelName of modelsToTry) {
    try {
      console.log(`Testing: ${modelName}...`);
      const model = genAI.getGenerativeModel({ model: modelName });

      const result = await model.generateContent("Say hello in one word");
      const response = await result.response;
      const text = response.text();

      console.log(`✅ ${modelName} WORKS!`);
      console.log(`   Response: ${text.substring(0, 50)}\n`);

      // Found a working model, update the config
      console.log(`\n🎉 SUCCESS! Use this model: ${modelName}`);
      return modelName;
    } catch (error: any) {
      if (error.message?.includes('429') || error.message?.includes('quota')) {
        console.log(`❌ ${modelName} - Quota exceeded\n`);
      } else if (error.message?.includes('404') || error.message?.includes('not found')) {
        console.log(`❌ ${modelName} - Not found\n`);
      } else {
        console.log(`❌ ${modelName} - Error: ${error.message?.substring(0, 100)}\n`);
      }
    }
  }

  console.log("\n⚠️  All models failed. You need a new API key.");
  return null;
}

testModels().catch(console.error);
