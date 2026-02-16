import { config } from "dotenv";
config(); // Load .env file

console.log("QDRANT_URL:", process.env.QDRANT_URL);

// Import the same way as the RAG functions
import { searchChunks } from "./src/lib/qdrant";
import { generateEmbedding } from "./src/lib/embeddings";

async function testSearchChunks() {
  console.log("Testing searchChunks function directly...");

  const query = "What is ROS 2?";
  console.log(`Query: "${query}"`);

  try {
    // Generate embedding for the query
    const queryEmbedding = await generateEmbedding(query);
    console.log(`Generated embedding with ${queryEmbedding.length} dimensions`);

    // Search for similar chunks
    const results = await searchChunks(queryEmbedding, 5);
    console.log(`Found ${results.length} results`);

    if (results.length > 0) {
      console.log("\nFirst result:");
      console.log(`ID: ${results[0].id}`);
      console.log(`Score: ${results[0].score}`);
      console.log(`Chapter: ${results[0].payload.chapter}`);
      console.log(`Text preview: ${results[0].payload.text.substring(0, 100)}...`);
    }
  } catch (error) {
    console.error("Error during searchChunks test:", error);
    console.error("Error message:", error.message);
    if (error.cause) {
      console.error("Error cause:", error.cause);
    }
  }
}

testSearchChunks().catch(console.error);