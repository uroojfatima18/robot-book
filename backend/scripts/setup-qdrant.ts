import { config } from "dotenv";
config(); // Loads from .env by default

async function main() {
  // Dynamic import AFTER dotenv loads the environment variables
  const { ensureCollection, collectionExists, getCollectionStats } = await import("../src/lib/qdrant");
  console.log("🔧 Setting up Qdrant collection...\n");

  const exists = await collectionExists();
  if (exists) {
    console.log("✅ Collection 'book_chunks' already exists");
    const stats = await getCollectionStats();
    console.log(`   Vectors: ${stats.vectorsCount}`);
    console.log(`   Points: ${stats.pointsCount}`);
    return;
  }

  console.log("📦 Creating collection 'book_chunks'...");
  await ensureCollection();
  console.log("✅ Collection created successfully!");
  console.log("   Vector size: 1536 (text-embedding-3-small)");
  console.log("   Distance: Cosine");
  console.log("   Indexes: chapter, pageId");
}

main().catch((error) => {
  console.error("❌ Error setting up Qdrant:", error.message || error);
  if (error.message?.includes("insufficient permissions")) {
    console.log("\n📝 Manual Setup Required:");
    console.log("   1. Go to your Qdrant Cloud dashboard");
    console.log("   2. Create a collection named 'book_chunks'");
    console.log("   3. Set vector size to 768 (for Gemini embeddings)");
    console.log("   4. Set distance function to 'Cosine'");
    console.log("   5. Create payload indexes for 'chapter' and 'pageId' fields");
    console.log("   6. Run this script again");
  }
  process.exit(1);
});
