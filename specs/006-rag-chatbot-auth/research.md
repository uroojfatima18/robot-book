# Research: RAG Chatbot with Authentication

**Feature**: 006-rag-chatbot-auth
**Date**: 2026-01-02
**Status**: Complete

## Executive Summary

This document captures research findings for building an integrated RAG chatbot with authentication for the robotics book UI. Key technology decisions have been validated and implementation patterns documented.

---

## 1. Authentication: Better Auth

### Decision
Use **Better Auth** library for email/password authentication with JWT tokens.

### Rationale
- Framework-agnostic TypeScript library with comprehensive feature set
- Built-in email/password authentication (no plugin needed)
- Native PostgreSQL adapter for Neon integration
- Plugin ecosystem for JWT, organizations, and email verification
- Active maintenance and high documentation quality

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| NextAuth.js | Less flexible, more opinionated, harder to customize token lifetimes |
| Auth0 | External dependency, cost at scale, vendor lock-in |
| Custom JWT | Significant development effort, security risks |

### Implementation Pattern

```typescript
// lib/auth.ts
import { betterAuth } from "better-auth";
import { jwt } from "better-auth/plugins";

export const auth = betterAuth({
  database: {
    type: "postgresql",
    url: process.env.DATABASE_URL,
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days (refresh token)
    updateAge: 60 * 60 * 24,     // Refresh after 1 day
  },
  plugins: [jwt()],
  advanced: {
    database: {
      generateId: "uuid",
    },
  },
});
```

### JWT Token Configuration
- **Access Token**: 15 minutes (via JWT plugin, verified at `/api/auth/jwks`)
- **Refresh Token**: 7 days (session-based, automatic refresh)
- Tokens stored in HTTP-only cookies (SameSite=Lax)

---

## 2. Email Service: Resend

### Decision
Use **Resend** for transactional emails (verification, password reset).

### Rationale
- Developer-friendly API
- High deliverability
- Cost-effective for low volume
- Easy integration with Better Auth

### Implementation Pattern

```typescript
import { Resend } from "resend";

const resend = new Resend(process.env.RESEND_API_KEY);

// Better Auth email verification config
emailVerification: {
  sendOnSignUp: true,
  sendVerificationEmail: async ({ user, url }) => {
    await resend.emails.send({
      from: "noreply@roboticsbook.com",
      to: user.email,
      subject: "Verify your email - Robotics Book",
      html: `<a href="${url}">Click to verify your email</a>`,
    });
  },
},
```

---

## 3. Vector Database: Qdrant Cloud

### Decision
Use **Qdrant Cloud** for vector storage and similarity search.

### Rationale
- Native JavaScript/TypeScript client
- Cosine similarity with score threshold filtering
- Payload metadata for chunk context
- High performance for concurrent queries
- Managed cloud service (no infrastructure)

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| Pinecone | Higher cost, less flexible filtering |
| Weaviate | More complex setup, heavier runtime |
| pgvector | Less optimized for high-dimensional search at scale |

### Implementation Pattern

```typescript
import { QdrantClient } from "@qdrant/js-client-rest";

const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

// Create collection for 1536-dimension embeddings
await qdrant.createCollection("book_chunks", {
  vectors: {
    size: 1536,
    distance: "Cosine",
  },
  optimizers_config: {
    memmap_threshold: 20000,
  },
});

// Search with score threshold
const results = await qdrant.search("book_chunks", {
  vector: queryEmbedding,
  limit: 5,
  score_threshold: 0.7,
  with_payload: true,
});
```

### Metadata Schema
```typescript
interface ChunkPayload {
  chapter: string;
  source: string;
  location: string;
  page_id: string;
  text: string;
  token_count: number;
}
```

---

## 4. Embeddings: OpenAI text-embedding-3-small

### Decision
Use **OpenAI text-embedding-3-small** (1536 dimensions) for generating embeddings.

### Rationale
- Industry standard quality
- Cost-effective ($0.02/1M tokens)
- 1536 dimensions balances quality and storage
- Simple API integration

### Implementation Pattern

```typescript
import OpenAI from "openai";

const openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });

async function generateEmbedding(text: string): Promise<number[]> {
  const response = await openai.embeddings.create({
    model: "text-embedding-3-small",
    input: text,
  });
  return response.data[0].embedding;
}
```

---

## 5. LLM Provider: Google Gemini

### Decision
Use **Google Gemini** (gemini-2.5-flash) for chat completions with streaming.

### Rationale
- User preference (specified in requirements)
- Competitive performance
- Streaming support
- Cost-effective

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| OpenAI GPT-4 | User preference for Gemini |
| Claude | User preference for Gemini |

### Implementation Pattern

```typescript
import { GoogleGenAI } from "@google/genai";

const ai = new GoogleGenAI({ apiKey: process.env.GEMINI_API_KEY });

async function* streamChatResponse(
  context: string,
  question: string,
  history: Array<{ role: string; content: string }>
): AsyncGenerator<string> {
  const chat = ai.chats.create({
    model: "gemini-2.5-flash",
    history: history.map(msg => ({
      role: msg.role === "user" ? "user" : "model",
      parts: [{ text: msg.content }],
    })),
    systemInstruction: {
      parts: [{ text: `You are a helpful assistant for a robotics textbook.
Answer questions ONLY based on the following context from the book.
If the question cannot be answered from the context, respond with:
"This question is outside the provided material."

Context:
${context}` }],
    },
  });

  const stream = await chat.sendMessageStream({ message: question });
  for await (const chunk of stream) {
    yield chunk.text;
  }
}
```

---

## 6. Database: Neon PostgreSQL with Drizzle ORM

### Decision
Use **Neon PostgreSQL** with **Drizzle ORM** for relational data.

### Rationale
- Serverless-optimized (connection pooling)
- Better Auth native support
- Drizzle: lightweight, type-safe, fast cold starts
- Cost-effective for serverless deployment

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| Prisma | Slower cold starts in serverless |
| Raw SQL | Less type safety, more boilerplate |
| Supabase | Additional abstraction layer not needed |

### Connection Pattern

```typescript
import { drizzle } from "drizzle-orm/neon-http";
import { neon } from "@neondatabase/serverless";

const sql = neon(process.env.DATABASE_URL!);
export const db = drizzle(sql);
```

---

## 7. Rate Limiting

### Decision
Use **token bucket algorithm** in PostgreSQL for rate limiting (10 messages/minute).

### Rationale
- No additional infrastructure (Redis)
- Atomic operations with advisory locks
- Simple implementation
- Sufficient for expected load

### Implementation Pattern

```sql
CREATE TABLE rate_limits (
  user_id UUID PRIMARY KEY REFERENCES users(id),
  tokens DECIMAL(10, 4) DEFAULT 10.0,
  last_refill TIMESTAMP DEFAULT NOW()
);

CREATE OR REPLACE FUNCTION check_rate_limit(p_user_id UUID)
RETURNS BOOLEAN AS $$
DECLARE
  v_tokens DECIMAL;
  v_elapsed NUMERIC;
BEGIN
  PERFORM pg_advisory_xact_lock(hashtext(p_user_id::text));

  SELECT tokens, EXTRACT(EPOCH FROM (NOW() - last_refill))
  INTO v_tokens, v_elapsed
  FROM rate_limits WHERE user_id = p_user_id;

  -- Refill: 10 tokens per 60 seconds
  v_tokens := LEAST(10.0, v_tokens + (v_elapsed * 0.1667));

  IF v_tokens < 1.0 THEN
    RETURN FALSE;
  END IF;

  UPDATE rate_limits
  SET tokens = v_tokens - 1.0, last_refill = NOW()
  WHERE user_id = p_user_id;

  RETURN TRUE;
END;
$$ LANGUAGE plpgsql;
```

---

## 8. Chunking Strategy

### Decision
Chunk text into **400-700 tokens** with overlap.

### Rationale
- Optimal for retrieval granularity
- Fits within context limits when combining 5 chunks
- Preserves semantic coherence

### Implementation Pattern

```typescript
import { encode } from "gpt-tokenizer";

function chunkText(text: string, maxTokens = 500, overlap = 50): string[] {
  const tokens = encode(text);
  const chunks: string[] = [];

  for (let i = 0; i < tokens.length; i += maxTokens - overlap) {
    const chunkTokens = tokens.slice(i, i + maxTokens);
    chunks.push(decode(chunkTokens));
  }

  return chunks;
}
```

---

## 9. Context Truncation

### Decision
Truncate combined context to **2,000 tokens maximum**, prioritizing highest similarity scores.

### Rationale
- Prevents context overflow
- Maintains response quality
- Balances cost and comprehensiveness

### Implementation Pattern

```typescript
function truncateContext(chunks: ScoredChunk[], maxTokens = 2000): string {
  const sorted = chunks.sort((a, b) => b.score - a.score);
  let totalTokens = 0;
  const selected: string[] = [];

  for (const chunk of sorted) {
    const tokens = encode(chunk.text).length;
    if (totalTokens + tokens > maxTokens) break;
    selected.push(chunk.text);
    totalTokens += tokens;
  }

  return selected.join("\n\n---\n\n");
}
```

---

## Summary Table

| Component | Technology | Key Config |
|-----------|------------|------------|
| Authentication | Better Auth | Email/password + JWT |
| Email | Resend | Verification emails |
| Vector DB | Qdrant Cloud | 1536 dims, cosine, threshold 0.7 |
| Embeddings | OpenAI text-embedding-3-small | 1536 dimensions |
| LLM | Google Gemini (gemini-2.5-flash) | Streaming responses |
| Database | Neon PostgreSQL + Drizzle | Serverless pooling |
| Rate Limiting | PostgreSQL token bucket | 10 msg/min |
| Chunking | Custom | 400-700 tokens with overlap |

---

## Open Questions (Resolved)

- [x] Which embedding model? → text-embedding-3-small (1536 dims)
- [x] JWT lifetimes? → Access 15min, Refresh 7 days
- [x] Auth library? → Better Auth
- [x] Email service? → Resend
- [x] Similarity threshold? → 0.7
- [x] LLM provider? → Google Gemini (user specified)
- [x] ORM? → Drizzle (serverless optimized)
