import { config } from "dotenv";
config(); // Load .env file

console.log("Testing RAG functionality across all 5 chapters...\n");

// Import the same way as the RAG functions
import("./src/lib/qdrant").then(async (qdrantModule) => {
  import("./src/lib/embeddings").then(async ({ generateEmbedding }) => {

    // Test queries for each chapter
    const testQueries = [
      { query: "What is ROS 2?", expectedChapter: "Chapter 1" },
      { query: "What is digital twin?", expectedChapter: "Chapter 2" },
      { query: "What is AI robot brain?", expectedChapter: "Chapter 3" },
      { query: "What is workflow orchestration?", expectedChapter: "Chapter 4" },
      { query: "What is adaptive robotics?", expectedChapter: "Chapter 5" }
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

    console.log("All tests completed!");
  });
}).catch((error) => {
  console.error("Failed to import modules:", error);
});