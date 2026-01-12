console.log('🧪 Testing chatbot with new API key...\n');

fetch('http://localhost:3001/api/chat/public', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ message: 'What is ROS 2?' })
})
.then(async response => {
  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let fullResponse = '';
  let tokens = [];

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    const chunk = decoder.decode(value);
    fullResponse += chunk;

    // Extract token data
    const lines = chunk.split('\n');
    for (const line of lines) {
      if (line.startsWith('data: ')) {
        try {
          const data = JSON.parse(line.slice(6));
          if (data.type === 'token') {
            tokens.push(data.data);
          }
        } catch (e) {
          // Ignore parse errors
        }
      }
    }
  }

  const fullText = tokens.join('');

  if (fullText.includes('ROS') && !fullText.includes('Error') && !fullText.includes('quota')) {
    console.log('✅ SUCCESS! Chatbot is working!\n');
    console.log('Response from your book:');
    console.log('─'.repeat(60));
    console.log(fullText.substring(0, 400));
    if (fullText.length > 400) {
      console.log('...(truncated)');
    }
    console.log('─'.repeat(60));
    console.log('\n🎉 Your chatbot is now fully functional!');
    console.log('📍 Open http://localhost:3005 to use it in the browser');
  } else if (fullText.includes('quota') || fullText.includes('capacity')) {
    console.log('❌ API key still has quota issues');
    console.log('Response:', fullText.substring(0, 200));
  } else {
    console.log('⚠️  Unexpected response');
    console.log('Response:', fullText.substring(0, 300));
  }
})
.catch(err => {
  console.error('❌ Error:', err.message);
});
