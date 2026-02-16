import { config } from "dotenv";
config(); // Load .env file

console.log("Testing RAG system directly without LLM...\n");

// Test the RAG functionality directly
import("./src/lib/rag/retrieve").then(async (ragModule) => {
  const query = "What is ROS 2?";
  console.log(`Query: "${query}"`);

  try {
    const result = await ragModule.retrieveContext(query);
    console.log(`\nFound ${result.chunks.length} relevant chunks`);
    console.log(`Total tokens: ${result.totalTokens}`);

    if (result.chunks.length > 0) {
      console.log("\nTop result:");
      console.log(`Chapter: ${result.chunks[0].payload.chapter}`);
      console.log(`Score: ${result.chunks[0].score}`);
      console.log(`Text preview: ${result.chunks[0].payload.text.substring(0, 200)}...`);

      console.log("\nAll relevant chapters found:");
      const uniqueChapters = [...new Set(result.chunks.map(chunk => chunk.payload.chapter))];
      uniqueChapters.forEach((chapter, index) => {
        console.log(`  ${index + 1}. ${chapter}`);
      });
    } else {
      console.log("No relevant content found in the vector database.");
    }
  } catch (error) {
    console.error("Error during RAG test:", error);
  }
}).catch((error) => {
  console.error("Failed to import RAG module:", error);
});