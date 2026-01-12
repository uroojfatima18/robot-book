import { config } from "dotenv";
config(); // Load .env file

console.log("Original QDRANT_URL:", process.env.QDRANT_URL);

// Extract just the base URL without the port for HTTPS
let fixedUrl = process.env.QDRANT_URL;
if (fixedUrl && fixedUrl.includes('https://') && fixedUrl.endsWith(':6333')) {
  fixedUrl = fixedUrl.substring(0, fixedUrl.length - 5); // Remove ':6333'
  console.log("Fixed QDRANT_URL:", fixedUrl);
}

import { QdrantClient } from "@qdrant/js-client-rest";

// Initialize Qdrant client with the fixed URL
const qdrant = new QdrantClient({
  url: fixedUrl!,
  apiKey: process.env.QDRANT_API_KEY!,
  checkVersionCompatibility: false, // Skip version compatibility check
});

console.log("Testing with fixed URL...");

async function testConnection() {
  try {
    // Try to search with a test query
    const testEmbedding = Array(384).fill(0.1); // Create a test embedding
    const results = await qdrant.search("book_chunks_local", {
      vector: testEmbedding,
      limit: 1,
    });

    console.log("Search successful! Found", results.length, "results");

    if (results.length > 0) {
      console.log("First result score:", results[0].score);
    }
  } catch (error) {
    console.error("Search failed:", error.message);
  }
}

testConnection();