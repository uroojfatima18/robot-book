import { config } from "dotenv";
config(); // Load .env file

console.log("Testing Chapter 3 specific content...\n");

// Import the same way as the RAG functions
import("./src/lib/qdrant").then(async (qdrantModule) => {
  import("./src/lib/embeddings").then(async ({ generateEmbedding }) => {

    // Test specific queries for Chapter 3
    const testQueries = [
      { query: "Isaac AI brain", expectedChapter: "Chapter 3" },
      { query: "AI robot brain architecture", expectedChapter: "Chapter 3" },
      { query: "Isaac Sim", expectedChapter: "Chapter 3" },
      { query: "AI-powered robot brain", expectedChapter: "Chapter 3" }
    ];

    for (const test of testQueries) {
      console.log(`Testing query: "${test.query}"`);
      const embedding = await generateEmbedding(test.query);
      const results = await qdrantModule.searchChunks(embedding, 1);

      if (results.length > 0) {
        console.log(`  ✓ Found result from: ${results[0].payload.chapter}`);
        console.log(`  Score: ${results[0].score}`);
        console.log(`  Text preview: ${results[0].payload.text.substring(0, 100)}...`);
      } else {
        console.log(`  ✗ No results found`);
      }
      console.log("");
    }

    console.log("Chapter 3 tests completed!");
  });
}).catch((error) => {
  console.error("Failed to import modules:", error);
});