import { config } from "dotenv";
config(); // Load .env file

console.log("Environment variables loaded:");
console.log("QDRANT_URL:", process.env.QDRANT_URL);
console.log("NODE_ENV:", process.env.NODE_ENV);

// Now let's check how the qdrant module is initialized
console.log("\nImporting qdrant module...");
import("./src/lib/qdrant").then((qdrantModule) => {
  console.log("Qdrant module imported successfully");
  console.log("Qdrant client type:", typeof qdrantModule.qdrant);

  // Let's check if we can access the client directly
  if (qdrantModule.qdrant) {
    console.log("Qdrant client exists");

    // Test the searchChunks function
    console.log("\nTesting searchChunks function...");
    import("./src/lib/embeddings").then(async ({ generateEmbedding }) => {
      const query = "What is ROS 2?";
      const embedding = await generateEmbedding(query);
      console.log(`Generated embedding with ${embedding.length} dimensions`);

      try {
        const results = await qdrantModule.searchChunks(embedding, 1);
        console.log(`Search successful! Found ${results.length} results`);
        if (results.length > 0) {
          console.log("First result score:", results[0].score);
          console.log("First result chapter:", results[0].payload.chapter);
        }
      } catch (error) {
        console.error("Search failed:", error.message);
        console.error("Error stack:", error.stack);
      }
    });
  }
}).catch((error) => {
  console.error("Failed to import qdrant module:", error);
});