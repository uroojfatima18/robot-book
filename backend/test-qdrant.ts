import { config } from "dotenv";
config(); // Load .env file

console.log("QDRANT_URL:", process.env.QDRANT_URL);
console.log("QDRANT_API_KEY exists:", !!process.env.QDRANT_API_KEY);

import { QdrantClient } from "@qdrant/js-client-rest";

// Initialize Qdrant client with logging
console.log("Creating Qdrant client with URL:", process.env.QDRANT_URL);

const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL!,
  apiKey: process.env.QDRANT_API_KEY!,
  checkVersionCompatibility: false, // Skip version compatibility check
});

console.log("Qdrant client created. Testing connection...");

async function testConnection() {
  try {
    // Try to get collection info to test the connection
    const info = await qdrant.getCollections();
    console.log("Connection successful! Collections:", info.collections.length);

    // Check if our collection exists
    try {
      const collectionInfo = await qdrant.getCollection("book_chunks_local");
      console.log("Collection info:", collectionInfo);
    } catch (err) {
      console.log("Collection 'book_chunks_local' doesn't exist yet:", err.message);
    }
  } catch (error) {
    console.error("Connection failed:", error.message);
    console.error("Error cause:", error.cause);
  }
}

testConnection();