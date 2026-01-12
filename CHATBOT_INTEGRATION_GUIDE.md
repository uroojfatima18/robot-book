# Chatbot Integration Guide

## Overview

Your Docusaurus frontend (`my-website/`) is now connected to the backend RAG chatbot API (`backend/`). The chatbot uses:
- **Google Gemini 2.0 Flash** for LLM responses
- **Qdrant Cloud** for vector search
- **Local embeddings** (Supabase/gte-small) for query processing
- **RAG pipeline** to retrieve relevant book content before answering

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Docusaurus Frontend                       │
│                    (Port 3005 or 3000)                       │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Chatbot Widget (Right Side)                         │  │
│  │  - useChatAPI hook                                   │  │
│  │  - SSE streaming                                     │  │
│  │  - Real-time responses                               │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ HTTP POST
                            │ /api/chat/public
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    Backend API Server                        │
│                       (Port 3001)                            │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Public Chat Endpoint                                │  │
│  │  - No auth required                                  │  │
│  │  - Rate limiting (20 req/min per IP)                │  │
│  │  - CORS enabled                                      │  │
│  └──────────────────────────────────────────────────────┘  │
│                            │                                 │
│                            ▼                                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  RAG Pipeline                                        │  │
│  │  1. Generate embedding (local model)                │  │
│  │  2. Search Qdrant (top-5 chunks)                    │  │
│  │  3. Retrieve context (max 2000 tokens)              │  │
│  │  4. Stream Gemini response                          │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                    ┌───────────────┐
                    │  Qdrant Cloud │
                    │  (Vector DB)  │
                    └───────────────┘
```

---

## Setup Instructions

### 1. Backend Setup

#### Prerequisites
- Node.js 18+
- PostgreSQL database (Neon)
- Qdrant Cloud account
- Google Gemini API key

#### Environment Variables

The backend already has a `.env` file configured. Verify these are set:

```bash
cd backend

# Check environment variables
cat .env
```

Required variables:
- `DATABASE_URL` - Neon PostgreSQL connection string ✅
- `QDRANT_URL` - Qdrant Cloud endpoint ✅
- `QDRANT_API_KEY` - Qdrant API key ✅
- `GEMINI_API_KEY` - Google Gemini API key ✅
- `BETTER_AUTH_SECRET` - Auth secret ✅
- `BETTER_AUTH_URL` - Auth URL (http://localhost:3001) ✅

#### Install Dependencies

```bash
cd backend
npm install
```

#### Ingest Book Content (If Not Done)

```bash
# Setup Qdrant collection
npm run qdrant:setup

# Ingest book chapters into vector database
npm run ingest:book
```

This will:
1. Create the `book_chunks_local` collection in Qdrant
2. Process all markdown files in `chapters/` directory
3. Generate embeddings using local model
4. Upload chunks to Qdrant

#### Start Backend Server

```bash
cd backend
npm run dev
```

Server will start on **http://localhost:3001**

Verify it's running:
```bash
curl http://localhost:3001/api/health
```

Expected response:
```json
{"status":"ok","timestamp":"2026-01-09T..."}
```

---

### 2. Frontend Setup

#### Install Dependencies

```bash
cd my-website
npm install
```

#### Start Docusaurus

```bash
cd my-website
npm start
```

Server will start on **http://localhost:3000** (or 3005 if configured)

---

## Testing the Integration

### Test 1: Basic Connection

1. Open http://localhost:3000 in your browser
2. Click the floating robot avatar button (bottom-right)
3. Chatbot widget should open with welcome message

**Expected Result:**
- Widget opens smoothly
- Welcome message displays with topic list
- Status shows "Online"

### Test 2: Simple Query

Type in the chatbot:
```
What is ROS 2?
```

**Expected Behavior:**
1. Message appears immediately in chat
2. Status changes to "Thinking..."
3. Response streams in token-by-token with blinking cursor
4. Full response appears from book content
5. Status returns to "Online"

**Expected Response Type:**
The bot should answer based on book content about ROS 2, mentioning topics like:
- Robot Operating System
- Nodes, topics, services
- Communication patterns
- Nervous system analogy

### Test 3: Streaming Visualization

Type a longer question:
```
Explain how digital twins work in robotics and their benefits
```

**Expected Behavior:**
- Response streams in real-time
- Cursor blinks at the end of streaming text
- Smooth scrolling to bottom
- No lag or freezing

### Test 4: Out-of-Scope Query

Type:
```
What's the weather today?
```

**Expected Response:**
```
I couldn't find relevant information about that in the book.
Please ask about ROS 2, digital twins, AI robotics, navigation,
SLAM, or other topics covered in the robotics chapters.
```

### Test 5: Error Handling

1. Stop the backend server (Ctrl+C)
2. Try sending a message

**Expected Behavior:**
- Error message appears in red box
- User message is removed from chat
- Can retry after restarting backend

### Test 6: Rate Limiting

Send 21 messages rapidly (within 1 minute)

**Expected Behavior:**
- First 20 messages work fine
- 21st message returns rate limit error
- Error message: "Rate limit exceeded"

---

## Troubleshooting

### Issue: "Failed to send message"

**Possible Causes:**
1. Backend server not running
2. Wrong port configuration
3. CORS issues

**Solutions:**
```bash
# Check backend is running
curl http://localhost:3001/api/health

# Check backend logs
cd backend
npm run dev

# Verify CORS in backend/src/app/api/chat/public/route.ts
# Should have: "Access-Control-Allow-Origin": "*"
```

### Issue: "No response body"

**Cause:** SSE streaming not working

**Solution:**
- Check browser console for errors
- Verify backend is sending SSE format
- Test endpoint directly:
```bash
curl -X POST http://localhost:3001/api/chat/public \
  -H "Content-Type: application/json" \
  -d '{"message":"test"}'
```

### Issue: Bot says "couldn't find relevant information"

**Possible Causes:**
1. Qdrant collection empty
2. Embeddings not generated
3. Similarity threshold too high

**Solutions:**
```bash
# Check Qdrant collection
cd backend
npm run qdrant:setup

# Re-ingest book content
npm run ingest:book

# Check collection stats in backend logs
```

### Issue: Slow responses

**Possible Causes:**
1. First request loads embedding model (takes 5-10 seconds)
2. Qdrant Cloud latency
3. Gemini API rate limits

**Solutions:**
- First request is always slower (model loading)
- Subsequent requests should be fast (1-3 seconds)
- Check Gemini API quota in Google Cloud Console

### Issue: CORS errors in browser console

**Error:**
```
Access to fetch at 'http://localhost:3001/api/chat/public'
from origin 'http://localhost:3000' has been blocked by CORS policy
```

**Solution:**
The backend already has CORS configured. If you see this:

1. Check backend is running on port 3001
2. Verify `useChatAPI.js` has correct API_BASE_URL
3. Check backend logs for CORS errors

---

## Configuration

### Change Backend Port

If backend needs to run on different port:

1. Update `backend/package.json`:
```json
"dev": "next dev -p 3002"
```

2. Update `my-website/src/hooks/useChatAPI.js`:
```javascript
const API_BASE_URL = "http://localhost:3002";
```

### Change Frontend Port

If Docusaurus needs different port:

```bash
cd my-website
npm start -- --port 3005
```

Update backend CORS if needed:
```typescript
// backend/src/app/api/chat/public/route.ts
"Access-Control-Allow-Origin": "http://localhost:3005"
```

---

## Features

### ✅ Implemented

- [x] Real-time streaming responses
- [x] RAG-based answers from book content
- [x] Rate limiting (20 req/min)
- [x] Error handling with user feedback
- [x] Loading states
- [x] Auto-scroll to latest message
- [x] Cyberpunk-themed UI
- [x] Welcome message with topic suggestions
- [x] Out-of-scope query detection

### 🚧 Not Yet Implemented

- [ ] User authentication (Better Auth integration)
- [ ] Conversation history persistence
- [ ] Multi-turn conversations with context
- [ ] Chapter filtering
- [ ] Source citations (which chapter answered from)
- [ ] Copy response button
- [ ] Regenerate response
- [ ] Clear conversation button

---

## Next Steps: Phase 2 (Optional)

### Add Better Auth Integration

To enable full features (conversation history, user accounts):

1. **Install Better Auth in Frontend:**
```bash
cd my-website
npm install better-auth
```

2. **Create Auth Client:**
```javascript
// my-website/src/lib/auth-client.js
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3001"
});
```

3. **Update Chatbot to Use Authenticated Endpoint:**
```javascript
// Change endpoint from /api/chat/public to /api/chat
// Add auth headers
```

4. **Benefits:**
- Conversation history saved to database
- Resume conversations
- Higher rate limits
- Personalized experience

---

## API Reference

### Public Chat Endpoint

**Endpoint:** `POST /api/chat/public`

**Request:**
```json
{
  "message": "What is ROS 2?"
}
```

**Response:** Server-Sent Events (SSE)

**Event Types:**

1. **Token Event** (streaming response):
```
data: {"type":"token","data":"ROS 2 is"}
```

2. **Done Event** (end of stream):
```
data: {"type":"done"}
```

3. **Error Event**:
```
data: {"type":"error","message":"Rate limit exceeded"}
```

**Rate Limits:**
- 20 requests per minute per IP
- Returns 429 status code when exceeded

**CORS:**
- Allows all origins (`*`)
- Methods: POST, OPTIONS
- Headers: Content-Type

---

## Performance Metrics

**Expected Response Times:**

| Metric | First Request | Subsequent Requests |
|--------|--------------|---------------------|
| Embedding Generation | 5-10s (model load) | 100-300ms |
| Vector Search | 200-500ms | 200-500ms |
| LLM Streaming | 1-3s | 1-3s |
| **Total** | **6-14s** | **1.5-4s** |

**Token Streaming:**
- ~50-100 tokens/second
- Visible streaming effect in UI
- Smooth user experience

---

## Security Notes

### Current Setup (Public Endpoint)

⚠️ **Security Considerations:**
- No authentication required
- IP-based rate limiting only
- No conversation persistence
- Suitable for demo/testing only

### Production Recommendations

For production deployment:

1. **Enable Authentication:**
   - Use Better Auth integration
   - Require verified email
   - User-based rate limiting

2. **Add API Key:**
   - Protect public endpoint with API key
   - Rotate keys regularly

3. **Enhanced Rate Limiting:**
   - Use Redis for distributed rate limiting
   - Different tiers for authenticated users

4. **Monitoring:**
   - Log all queries
   - Track usage metrics
   - Alert on abuse patterns

5. **Content Filtering:**
   - Validate input length
   - Filter malicious content
   - Sanitize responses

---

## Monitoring & Debugging

### Backend Logs

Watch backend logs for debugging:
```bash
cd backend
npm run dev

# Look for:
# - "📦 Loading embedding model..." (first request)
# - "✅ Embedding model loaded"
# - "Streaming error:" (errors)
```

### Browser Console

Open browser DevTools (F12) and check:
- Network tab: See SSE stream
- Console: JavaScript errors
- Application tab: Check localStorage

### Test Backend Directly

```bash
# Test health endpoint
curl http://localhost:3001/api/health

# Test chat endpoint
curl -X POST http://localhost:3001/api/chat/public \
  -H "Content-Type: application/json" \
  -d '{"message":"What is ROS 2?"}' \
  --no-buffer

# Should see streaming response
```

---

## Summary

✅ **Integration Complete!**

Your Docusaurus frontend now has a fully functional RAG chatbot that:
- Connects to your backend API
- Streams responses in real-time
- Answers questions from book content
- Handles errors gracefully
- Provides excellent UX with cyberpunk theme

**To Start Using:**
1. `cd backend && npm run dev` (port 3001)
2. `cd my-website && npm start` (port 3000)
3. Open http://localhost:3000
4. Click robot avatar (bottom-right)
5. Ask questions about robotics!

**Test Questions:**
- "What is ROS 2?"
- "Explain digital twins"
- "How does SLAM work?"
- "What is the robot nervous system?"
- "Tell me about adaptive robotics"

Enjoy your AI-powered robotics book assistant! 🤖📚
