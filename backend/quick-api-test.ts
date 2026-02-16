/**
 * Simple Direct API Test
 */

async function quickTest() {
  console.log('Testing API endpoints...\n');

  // Test health endpoint
  try {
    const response = await fetch('http://localhost:3001/api/health', {
      signal: AbortSignal.timeout(5000)
    });
    const data = await response.json();
    console.log('✅ Health endpoint:', data);
  } catch (error: any) {
    console.log('❌ Health endpoint failed:', error.message);
  }

  // Test public chat with timeout
  try {
    console.log('\nTesting public chat endpoint...');
    const response = await fetch('http://localhost:3001/api/chat/public', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message: 'Hello' }),
      signal: AbortSignal.timeout(10000)
    });

    console.log('Status:', response.status);
    console.log('Content-Type:', response.headers.get('content-type'));

    if (response.ok) {
      console.log('✅ Public chat endpoint is accessible');
    } else {
      console.log('❌ Public chat endpoint returned:', response.status);
    }
  } catch (error: any) {
    console.log('❌ Public chat endpoint failed:', error.message);
  }
}

quickTest();
