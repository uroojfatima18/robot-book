import { retrieveContext, hasRelevantContent } from "./src/lib/rag/retrieve";

async function testRAG() {
  console.log("Testing RAG functionality...");

  // Test a query related to ROS 2
  const query = "What is ROS 2?";

  console.log(`\nQuery: "${query}"`);

  try {
    // Check if we have relevant content
    const hasContent = await hasRelevantContent(query);
    console.log(`Has relevant content: ${hasContent}`);

    if (hasContent) {
      // Retrieve context
      const result = await retrieveContext(query);
      console.log(`Found ${result.chunks.length} chunks`);
      console.log(`Total tokens: ${result.totalTokens}`);
      console.log(`Context text length: ${result.contextText.length}`);

      if (result.chunks.length > 0) {
        console.log("\nFirst chunk preview:");
        console.log(result.chunks[0].payload.text.substring(0, 200) + "...");
        console.log(`Score: ${result.chunks[0].score}`);
        console.log(`Chapter: ${result.chunks[0].payload.chapter}`);
      }
    } else {
      console.log("No relevant content found in the book.");
    }
  } catch (error) {
    console.error("Error during RAG test:", error);
  }
}

testRAG().catch(console.error);