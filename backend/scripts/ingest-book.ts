import { config } from "dotenv";
config(); // Loads from .env by default

import * as fs from "fs";
import * as path from "path";

const CHAPTERS_DIR = path.join(__dirname, "../../chapters"); // Go up to robot-book/chapters
const WEBSITE_CHAPTERS_DIR = path.join(__dirname, "../../my-website/docs"); // Go up to robot-book/my-website/docs

interface ChapterConfig {
  dir: string;
  name: string;
  sourceDir: string; // Add source directory to handle different chapter locations
}

const CHAPTERS: ChapterConfig[] = [
  { dir: "01-ros2-nervous-system", name: "Chapter 1: ROS 2 - The Robot's Nervous System", sourceDir: "chapters" },
  { dir: "02-digital-twin", name: "Chapter 2: Digital Twin - Virtual Robot Laboratory", sourceDir: "chapters" },
  { dir: "003-ai-robot-brain", name: "Chapter 3: AI Robot Brain", sourceDir: "chapters" },
  { dir: "chapter-04-workflow-orchestration", name: "Chapter 4: Workflow Orchestration", sourceDir: "my-website/docs" },
  { dir: "chapter-05-adaptive-robotics", name: "Chapter 5: Adaptive Robotics", sourceDir: "my-website/docs" },
];

// Debug: Show the chapters directory path
console.log(`Looking for chapters in: ${CHAPTERS_DIR}`);

function readMarkdownFilesRecursive(dir: string, relativePath: string = ""): string[] {
  const content: string[] = [];

  if (!fs.existsSync(dir)) {
    return content;
  }

  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    const relPath = relativePath ? `${relativePath}/${entry.name}` : entry.name;

    if (entry.isDirectory() && !entry.name.startsWith("_") && !entry.name.startsWith(".")) {
      // Recurse into subdirectories
      content.push(...readMarkdownFilesRecursive(fullPath, relPath));
    } else if (entry.isFile() && entry.name.endsWith(".md") && entry.name !== "README.md") {
      const text = fs.readFileSync(fullPath, "utf-8");
      content.push(`## ${relPath}\n\n${text}`);
    }
  }

  return content;
}

function readMarkdownFiles(dir: string): string {
  if (!fs.existsSync(dir)) {
    console.log(`  ⚠️ Directory not found: ${dir}`);
    return "";
  }

  const content = readMarkdownFilesRecursive(dir);
  return content.join("\n\n---\n\n");
}

async function main() {
  // Dynamic imports AFTER dotenv loads the environment variables
  const { ingestContent } = await import("../src/lib/rag/ingest");
  const { ensureCollection } = await import("../src/lib/qdrant");

  console.log("📚 Ingesting book content into Qdrant...\n");

  // Ensure collection exists
  await ensureCollection();
  console.log("✅ Qdrant collection ready\n");

  let totalChunks = 0;
  let totalTokens = 0;

  for (const chapter of CHAPTERS) {
    let chapterDir: string;
    if (chapter.sourceDir === "chapters") {
      chapterDir = path.join(CHAPTERS_DIR, chapter.dir);
    } else if (chapter.sourceDir === "my-website/docs") {
      chapterDir = path.join(WEBSITE_CHAPTERS_DIR, chapter.dir);
    } else {
      console.log(`   ⚠️ Unknown source directory for chapter: ${chapter.dir}\n`);
      continue;
    }

    console.log(`📖 Processing: ${chapter.name} from ${chapterDir}`);

    const content = readMarkdownFiles(chapterDir);

    if (!content) {
      console.log(`   Skipped (no content)\n`);
      continue;
    }

    const result = await ingestContent(content, {
      chapter: chapter.name,
      source: chapter.dir,
    });

    console.log(`   ✅ Created ${result.chunksCreated} chunks (${result.totalTokens} tokens)\n`);
    totalChunks += result.chunksCreated;
    totalTokens += result.totalTokens;
  }

  console.log("═".repeat(50));
  console.log(`\n🎉 Ingestion complete!`);
  console.log(`   Total chunks: ${totalChunks}`);
  console.log(`   Total tokens: ${totalTokens}`);
}

main().catch((error) => {
  console.error("❌ Error ingesting book:", error);
  process.exit(1);
});
