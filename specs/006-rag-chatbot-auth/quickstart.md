# Quickstart: RAG Chatbot with Authentication

**Feature**: 006-rag-chatbot-auth
**Time to first run**: ~15 minutes

## Prerequisites

- Node.js 18+ and pnpm
- Neon PostgreSQL account (free tier works)
- Qdrant Cloud account (free tier works)
- OpenAI API key (for embeddings)
- Google AI API key (for Gemini)
- Resend API key (for emails)

---

## 1. Clone and Install

```bash
# Navigate to project
cd robot-book

# Install dependencies
pnpm install
```

---

## 2. Environment Setup

Create `.env.local` in the project root:

```env
# Database (Neon PostgreSQL)
DATABASE_URL="postgresql://user:pass@ep-xxx.us-east-1.aws.neon.tech/neondb?sslmode=require"

# Better Auth
BETTER_AUTH_SECRET="your-32-char-secret-here"
BETTER_AUTH_URL="http://localhost:3000"

# Vector Database (Qdrant Cloud)
QDRANT_URL="https://xxx.us-east-1.aws.cloud.qdrant.io:6333"
QDRANT_API_KEY="your-qdrant-api-key"

# OpenAI (for embeddings only)
OPENAI_API_KEY="sk-..."

# Google Gemini (for chat completions)
GEMINI_API_KEY="your-gemini-api-key"

# Email (Resend)
RESEND_API_KEY="re_..."
EMAIL_FROM="noreply@yourdomain.com"
```

---

## 3. Database Setup

```bash
# Generate and run migrations
pnpm db:generate
pnpm db:push

# Verify tables created
pnpm db:studio
```

This creates:
- `users` - User accounts (Better Auth)
- `sessions` - Authentication sessions
- `conversations` - Chat conversations
- `messages` - Conversation messages
- `rate_limits` - Rate limiting tokens

---

## 4. Vector Database Setup

```bash
# Run the Qdrant collection setup script
pnpm qdrant:setup
```

Or manually via Qdrant Cloud dashboard:
1. Create collection `book_chunks`
2. Set vector size: `1536`
3. Set distance: `Cosine`

---

## 5. Ingest Sample Content

```bash
# Ingest a sample chapter for testing
pnpm ingest:sample
```

Or via API (after starting the server):

```bash
curl -X POST http://localhost:3000/api/admin/ingest \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <admin-token>" \
  -d '{
    "content": "ROS 2 is the second generation of the Robot Operating System...",
    "chapter": "Chapter 2: ROS 2 Fundamentals"
  }'
```

---

## 6. Run Development Server

```bash
pnpm dev
```

Open [http://localhost:3000](http://localhost:3000)

---

## 7. Test the Flow

### Create Account
1. Navigate to `/signup`
2. Enter email and password
3. Check email for verification link
4. Click link to verify

### Test Chatbot
1. Log in at `/login`
2. Navigate to the book content
3. Click "Ask the Chatbot" or select text
4. Ask: "How do ROS 2 nodes communicate?"

### Test Rate Limiting
Send 11 messages within 1 minute - the 11th should be blocked.

---

## Project Structure

```
src/
├── app/
│   ├── api/
│   │   ├── auth/[...all]/    # Better Auth handler
│   │   ├── chat/             # Chat endpoints
│   │   ├── conversations/    # History endpoints
│   │   └── admin/            # Admin endpoints
│   ├── (auth)/               # Auth pages (login, signup)
│   └── (main)/               # Main app pages
├── lib/
│   ├── auth.ts               # Better Auth config
│   ├── db/
│   │   ├── index.ts          # Drizzle client
│   │   └── schema.ts         # Database schema
│   ├── qdrant.ts             # Qdrant client
│   ├── gemini.ts             # Gemini chat client
│   └── openai.ts             # OpenAI embeddings
├── components/
│   ├── chat/                 # Chatbot UI components
│   └── auth/                 # Auth form components
└── types/                    # Shared TypeScript types
```

---

## Key Commands

| Command | Description |
|---------|-------------|
| `pnpm dev` | Start development server |
| `pnpm build` | Build for production |
| `pnpm db:generate` | Generate Drizzle migrations |
| `pnpm db:push` | Push schema to database |
| `pnpm db:studio` | Open Drizzle Studio |
| `pnpm qdrant:setup` | Initialize Qdrant collection |
| `pnpm ingest:sample` | Ingest sample chapter |
| `pnpm test` | Run tests |

---

## Troubleshooting

### "Email not verified" error
- Check spam folder for verification email
- Verify Resend API key and sender domain
- Request new verification email at `/resend-verification`

### "No matching answer" for valid questions
- Verify content was ingested: check Qdrant dashboard
- Lower similarity threshold temporarily (0.6) to debug
- Check embedding dimensions match (1536)

### Rate limit not resetting
- Tokens refill at ~1 every 6 seconds
- Check `rate_limits` table for current token count
- Verify PostgreSQL function was created

### Connection pooling errors
- Use pooled connection string (with `-pooler` suffix)
- Set `max_connections` appropriately for serverless

---

## Next Steps

1. **Customize UI**: Modify `components/chat/` for your design
2. **Add chapters**: Use admin ingest to add book content
3. **Configure email templates**: Customize in `lib/auth.ts`
4. **Deploy**: See deployment guide for Vercel setup
