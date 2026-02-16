/**
 * Test Live API Endpoints
 * Tests the running server's endpoints
 */

async function testLiveEndpoints() {
  console.log('🌐 Testing Live API Endpoints\n');

  // Test 1: Health endpoint
  try {
    const healthResponse = await fetch('http://localhost:3001/api/health');
    const healthData = await healthResponse.json();
    console.log('✅ Health Endpoint:', healthResponse.status, healthData);
  } catch (error: any) {
    console.log('❌ Health Endpoint:', error.message);
  }

  // Test 2: Public chat endpoint (without streaming)
  try {
    console.log('\n📤 Testing Public Chat Endpoint...');
    const chatResponse = await fetch('http://localhost:3001/api/chat/public', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message: 'What is ROS 2?' }),
    });

    console.log('Status:', chatResponse.status);
    console.log('Headers:', Object.fromEntries(chatResponse.headers.entries()));

    if (chatResponse.ok) {
      const reader = chatResponse.body?.getReader();
      const decoder = new TextDecoder();
      let fullResponse = '';

      if (reader) {
        console.log('\n📥 Streaming response:');
        let chunkCount = 0;
        while (true) {
          const { done, value } = await reader.read();
          if (done) break;

          const chunk = decoder.decode(value);
          fullResponse += chunk;
          chunkCount++;

          // Parse SSE events
          const lines = chunk.split('\n');
          for (const line of lines) {
            if (line.startsWith('data: ')) {
              try {
                const data = JSON.parse(line.substring(6));
                if (data.type === 'token') {
                  process.stdout.write(data.data);
                } else if (data.type === 'done') {
                  console.log('\n\n✅ Stream completed');
                }
              } catch (e) {
                // Ignore parse errors
              }
            }
          }
        }
        console.log(`\nReceived ${chunkCount} chunks`);
      }
    } else {
      console.log('❌ Chat endpoint failed:', chatResponse.status, chatResponse.statusText);
    }
  } catch (error: any) {
    console.log('❌ Public Chat Endpoint:', error.message);
  }

  // Test 3: Check if authenticated endpoint requires auth
  try {
    console.log('\n🔒 Testing Authenticated Chat Endpoint (should require auth)...');
    const authChatResponse = await fetch('http://localhost:3001/api/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message: 'Test' }),
    });

    console.log('Status:', authChatResponse.status);
    if (authChatResponse.status === 401) {
      console.log('✅ Correctly requires authentication');
    } else {
      console.log('⚠️  Expected 401, got:', authChatResponse.status);
    }
  } catch (error: any) {
    console.log('❌ Authenticated Chat Endpoint:', error.message);
  }
}

testLiveEndpoints().catch(console.error);
