/**
 * Comprehensive Chatbot Validation Test Suite
 * Tests all components of the robotics book chatbot
 */

import * as dotenv from 'dotenv';
import * as path from 'path';

// Load environment variables
dotenv.config({ path: path.join(__dirname, '.env') });

interface TestResult {
  name: string;
  status: 'PASS' | 'FAIL' | 'WARN';
  message: string;
  details?: any;
}

const results: TestResult[] = [];

function logTest(result: TestResult) {
  results.push(result);
  const icon = result.status === 'PASS' ? '✅' : result.status === 'FAIL' ? '❌' : '⚠️';
  console.log(`${icon} ${result.name}: ${result.message}`);
  if (result.details) {
    console.log(`   Details: ${JSON.stringify(result.details, null, 2)}`);
  }
}

// ============================================================================
// PHASE 1: Environment & Configuration Validation
// ============================================================================
async function testEnvironmentVariables() {
  console.log('\n📋 PHASE 1: Environment & Configuration Validation\n');

  const requiredVars = [
    'DATABASE_URL',
    'BETTER_AUTH_SECRET',
    'BETTER_AUTH_URL',
    'QDRANT_URL',
    'QDRANT_API_KEY',
    'GEMINI_API_KEY',
  ];

  for (const varName of requiredVars) {
    const value = process.env[varName];
    if (!value) {
      logTest({
        name: `ENV: ${varName}`,
        status: 'FAIL',
        message: 'Not set or empty',
      });
    } else if (value.includes('your-') || value.includes('xxx')) {
      logTest({
        name: `ENV: ${varName}`,
        status: 'FAIL',
        message: 'Contains placeholder value',
      });
    } else {
      logTest({
        name: `ENV: ${varName}`,
        status: 'PASS',
        message: 'Configured',
        details: { length: value.length },
      });
    }
  }
}

// ============================================================================
// PHASE 2: Database Connectivity
// ============================================================================
async function testDatabaseConnection() {
  console.log('\n🗄️  PHASE 2: Database Connectivity\n');

  try {
    const { neon } = await import('@neondatabase/serverless');
    const sql = neon(process.env.DATABASE_URL!);

    // Test basic query
    const result = await sql`SELECT NOW() as current_time, version() as pg_version`;

    logTest({
      name: 'Database Connection',
      status: 'PASS',
      message: 'Successfully connected to Neon PostgreSQL',
      details: {
        timestamp: result[0].current_time,
        version: result[0].pg_version.substring(0, 50),
      },
    });

    // Check if tables exist
    const tables = await sql`
      SELECT table_name
      FROM information_schema.tables
      WHERE table_schema = 'public'
    `;

    logTest({
      name: 'Database Schema',
      status: 'PASS',
      message: `Found ${tables.length} tables`,
      details: { tables: tables.map((t: any) => t.table_name) },
    });

  } catch (error: any) {
    logTest({
      name: 'Database Connection',
      status: 'FAIL',
      message: error.message,
    });
  }
}

// ============================================================================
// PHASE 3: Vector Database (Qdrant) Validation
// ============================================================================
async function testQdrantConnection() {
  console.log('\n🔍 PHASE 3: Vector Database (Qdrant) Validation\n');

  try {
    const { QdrantClient } = await import('@qdrant/js-client-rest');

    const client = new QdrantClient({
      url: process.env.QDRANT_URL!,
      apiKey: process.env.QDRANT_API_KEY!,
      checkVersionCompatibility: false,
    });

    // Test connection
    const collections = await client.getCollections();

    logTest({
      name: 'Qdrant Connection',
      status: 'PASS',
      message: 'Successfully connected to Qdrant Cloud',
      details: {
        collections: collections.collections.map((c: any) => c.name),
      },
    });

    // Check for our collection
    const collectionName = 'book_chunks_local';
    try {
      const collectionInfo = await client.getCollection(collectionName);
      const info = collectionInfo as any;

      logTest({
        name: 'Qdrant Collection',
        status: 'PASS',
        message: `Collection '${collectionName}' exists`,
        details: {
          vectors_count: info.vectors_count || 0,
          points_count: info.points_count || 0,
          status: info.status,
        },
      });

      // Test search functionality
      if (info.points_count > 0) {
        const testVector = new Array(384).fill(0.1);
        const searchResults = await client.search(collectionName, {
          vector: testVector,
          limit: 1,
        });

        logTest({
          name: 'Qdrant Search',
          status: 'PASS',
          message: 'Vector search working',
          details: { results: searchResults.length },
        });
      } else {
        logTest({
          name: 'Qdrant Search',
          status: 'WARN',
          message: 'Collection is empty - no vectors to search',
        });
      }

    } catch (error: any) {
      logTest({
        name: 'Qdrant Collection',
        status: 'FAIL',
        message: `Collection '${collectionName}' not found`,
      });
    }

  } catch (error: any) {
    logTest({
      name: 'Qdrant Connection',
      status: 'FAIL',
      message: error.message,
    });
  }
}

// ============================================================================
// PHASE 4: AI Model Integration (Gemini)
// ============================================================================
async function testGeminiAPI() {
  console.log('\n🤖 PHASE 4: AI Model Integration (Gemini)\n');

  try {
    const { GoogleGenerativeAI } = await import('@google/generative-ai');

    const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);
    const model = genAI.getGenerativeModel({ model: 'gemini-2.0-flash' });

    // Test basic generation
    const result = await model.generateContent('Say "test successful" in exactly 2 words.');
    const response = result.response.text();

    logTest({
      name: 'Gemini API Key',
      status: 'PASS',
      message: 'API key is valid',
    });

    logTest({
      name: 'Gemini Text Generation',
      status: 'PASS',
      message: 'Successfully generated text',
      details: { response: response.substring(0, 100) },
    });

  } catch (error: any) {
    if (error.message?.includes('429') || error.message?.includes('quota')) {
      logTest({
        name: 'Gemini API',
        status: 'WARN',
        message: 'API quota exceeded - key is valid but rate limited',
      });
    } else if (error.message?.includes('API_KEY_INVALID')) {
      logTest({
        name: 'Gemini API',
        status: 'FAIL',
        message: 'Invalid API key',
      });
    } else {
      logTest({
        name: 'Gemini API',
        status: 'FAIL',
        message: error.message,
      });
    }
  }
}

// ============================================================================
// PHASE 5: Embeddings System
// ============================================================================
async function testEmbeddings() {
  console.log('\n🧮 PHASE 5: Embeddings System\n');

  try {
    const { generateEmbedding } = await import('./src/lib/embeddings');

    const testText = 'What is ROS 2?';
    const embedding = await generateEmbedding(testText);

    if (embedding.length === 384) {
      logTest({
        name: 'Local Embeddings',
        status: 'PASS',
        message: 'Successfully generated 384-dimensional embedding',
        details: { dimensions: embedding.length },
      });
    } else {
      logTest({
        name: 'Local Embeddings',
        status: 'FAIL',
        message: `Wrong embedding dimension: ${embedding.length} (expected 384)`,
      });
    }

  } catch (error: any) {
    logTest({
      name: 'Local Embeddings',
      status: 'FAIL',
      message: error.message,
    });
  }
}

// ============================================================================
// PHASE 6: RAG System Testing
// ============================================================================
async function testRAGSystem() {
  console.log('\n📚 PHASE 6: RAG System Testing\n');

  try {
    const { retrieveContext } = await import('./src/lib/rag/retrieve');

    const testQuery = 'What is ROS 2?';
    const result = await retrieveContext(testQuery);

    if (result.chunks.length > 0) {
      logTest({
        name: 'RAG Context Retrieval',
        status: 'PASS',
        message: `Retrieved ${result.chunks.length} relevant chunks`,
        details: {
          chunks: result.chunks.length,
          totalTokens: result.totalTokens,
          topScore: result.chunks[0]?.score,
        },
      });
    } else {
      logTest({
        name: 'RAG Context Retrieval',
        status: 'WARN',
        message: 'No relevant chunks found - collection may be empty',
      });
    }

  } catch (error: any) {
    logTest({
      name: 'RAG Context Retrieval',
      status: 'FAIL',
      message: error.message,
    });
  }
}

// ============================================================================
// PHASE 7: API Endpoint Testing
// ============================================================================
async function testAPIEndpoints() {
  console.log('\n🌐 PHASE 7: API Endpoint Testing\n');

  // Test health endpoint
  try {
    const response = await fetch('http://localhost:3001/api/health');
    const data = await response.json();

    if (response.ok) {
      logTest({
        name: 'Health Endpoint',
        status: 'PASS',
        message: 'Health check passed',
        details: data,
      });
    } else {
      logTest({
        name: 'Health Endpoint',
        status: 'FAIL',
        message: `HTTP ${response.status}`,
      });
    }
  } catch (error: any) {
    logTest({
      name: 'Health Endpoint',
      status: 'WARN',
      message: 'Server not running - start with: npm run dev',
    });
  }

  // Test public chat endpoint
  try {
    const response = await fetch('http://localhost:3001/api/chat/public', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message: 'What is ROS 2?' }),
    });

    if (response.ok) {
      logTest({
        name: 'Public Chat Endpoint',
        status: 'PASS',
        message: 'Endpoint is accessible',
      });
    } else {
      logTest({
        name: 'Public Chat Endpoint',
        status: 'FAIL',
        message: `HTTP ${response.status}`,
      });
    }
  } catch (error: any) {
    logTest({
      name: 'Public Chat Endpoint',
      status: 'WARN',
      message: 'Server not running',
    });
  }
}

// ============================================================================
// PHASE 8: End-to-End Test
// ============================================================================
async function testEndToEnd() {
  console.log('\n🎯 PHASE 8: End-to-End User Flow\n');

  try {
    // Import all necessary modules
    const { retrieveContext } = await import('./src/lib/rag/retrieve');
    const { generateChatResponse } = await import('./src/lib/gemini');

    const testQuestion = 'What is ROS 2?';

    // Step 1: Retrieve context
    const { contextText, chunks } = await retrieveContext(testQuestion);

    if (chunks.length === 0) {
      logTest({
        name: 'E2E: Context Retrieval',
        status: 'WARN',
        message: 'No context found - collection may be empty',
      });
      return;
    }

    logTest({
      name: 'E2E: Context Retrieval',
      status: 'PASS',
      message: `Retrieved ${chunks.length} chunks`,
    });

    // Step 2: Generate response
    const response = await generateChatResponse(contextText, testQuestion);

    if (response && response.length > 0) {
      logTest({
        name: 'E2E: Response Generation',
        status: 'PASS',
        message: 'Successfully generated response',
        details: {
          responseLength: response.length,
          preview: response.substring(0, 150) + '...',
        },
      });
    } else {
      logTest({
        name: 'E2E: Response Generation',
        status: 'FAIL',
        message: 'Empty response generated',
      });
    }

  } catch (error: any) {
    if (error.message?.includes('429') || error.message?.includes('quota')) {
      logTest({
        name: 'E2E Test',
        status: 'WARN',
        message: 'Gemini API quota exceeded',
      });
    } else {
      logTest({
        name: 'E2E Test',
        status: 'FAIL',
        message: error.message,
      });
    }
  }
}

// ============================================================================
// Main Test Runner
// ============================================================================
async function runAllTests() {
  console.log('╔════════════════════════════════════════════════════════════════╗');
  console.log('║     ROBOTICS BOOK CHATBOT - COMPREHENSIVE VALIDATION TEST     ║');
  console.log('╚════════════════════════════════════════════════════════════════╝');
  console.log(`\nStarted at: ${new Date().toISOString()}\n`);

  await testEnvironmentVariables();
  await testDatabaseConnection();
  await testQdrantConnection();
  await testGeminiAPI();
  await testEmbeddings();
  await testRAGSystem();
  await testAPIEndpoints();
  await testEndToEnd();

  // Generate summary report
  console.log('\n╔════════════════════════════════════════════════════════════════╗');
  console.log('║                        TEST SUMMARY                            ║');
  console.log('╚════════════════════════════════════════════════════════════════╝\n');

  const passed = results.filter(r => r.status === 'PASS').length;
  const failed = results.filter(r => r.status === 'FAIL').length;
  const warnings = results.filter(r => r.status === 'WARN').length;
  const total = results.length;

  console.log(`Total Tests: ${total}`);
  console.log(`✅ Passed: ${passed}`);
  console.log(`❌ Failed: ${failed}`);
  console.log(`⚠️  Warnings: ${warnings}`);
  console.log(`\nSuccess Rate: ${((passed / total) * 100).toFixed(1)}%`);

  // Critical issues
  const criticalFailures = results.filter(r =>
    r.status === 'FAIL' &&
    (r.name.includes('ENV:') || r.name.includes('Connection') || r.name.includes('API'))
  );

  if (criticalFailures.length > 0) {
    console.log('\n⚠️  CRITICAL ISSUES FOUND:\n');
    criticalFailures.forEach(f => {
      console.log(`   ❌ ${f.name}: ${f.message}`);
    });
  }

  // Recommendations
  console.log('\n📋 RECOMMENDATIONS:\n');

  if (failed === 0 && warnings === 0) {
    console.log('   ✅ All systems operational! Chatbot is ready to use.');
  } else {
    if (results.some(r => r.name.includes('Collection') && r.status === 'FAIL')) {
      console.log('   1. Create Qdrant collection: npm run qdrant:setup');
    }
    if (results.some(r => r.name.includes('Collection') && r.status === 'WARN' && r.message.includes('empty'))) {
      console.log('   2. Ingest book content: npm run ingest:book');
    }
    if (results.some(r => r.name.includes('Server not running'))) {
      console.log('   3. Start the development server: npm run dev');
    }
    if (results.some(r => r.name.includes('quota'))) {
      console.log('   4. Gemini API quota exceeded - wait or use a different API key');
    }
  }

  console.log(`\nCompleted at: ${new Date().toISOString()}\n`);
}

// Run tests
runAllTests().catch(console.error);
