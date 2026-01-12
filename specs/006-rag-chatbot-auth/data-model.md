# Data Model: RAG Chatbot with Authentication

**Feature**: 006-rag-chatbot-auth
**Date**: 2026-01-02

## Overview

This document defines the data model for the RAG chatbot with authentication system. The model spans two storage systems:
- **Neon PostgreSQL**: User accounts, sessions, conversations, rate limits
- **Qdrant Cloud**: Vector embeddings for book content chunks

---

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PostgreSQL (Neon)                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────────────┐       │
│  │   users     │     │  sessions   │     │   conversations     │       │
│  │─────────────│     │─────────────│     │─────────────────────│       │
│  │ id (PK)     │◄────│ user_id(FK) │     │ id (PK)             │       │
│  │ email       │     │ token       │     │ user_id (FK)────────┼──┐    │
│  │ name        │     │ expires_at  │     │ title               │  │    │
│  │ password    │     │ device_info │     │ created_at          │  │    │
│  │ role        │     │ created_at  │     │ updated_at          │  │    │
│  │ verified    │     └─────────────┘     └─────────────────────┘  │    │
│  │ created_at  │                                   │               │    │
│  └─────────────┘                                   │               │    │
│        │                                           │               │    │
│        │         ┌─────────────────────────────────┘               │    │
│        │         │                                                 │    │
│        │         ▼                                                 │    │
│        │    ┌─────────────────────┐                               │    │
│        │    │     messages        │                               │    │
│        │    │─────────────────────│                               │    │
│        │    │ id (PK)             │                               │    │
│        │    │ conversation_id(FK) │                               │    │
│        │    │ role                │                               │    │
│        │    │ content             │                               │    │
│        │    │ created_at          │                               │    │
│        │    └─────────────────────┘                               │    │
│        │                                                          │    │
│        │    ┌─────────────────────┐                               │    │
│        └───►│   rate_limits       │◄──────────────────────────────┘    │
│             │─────────────────────│                                    │
│             │ user_id (PK/FK)     │                                    │
│             │ tokens              │                                    │
│             │ last_refill         │                                    │
│             └─────────────────────┘                                    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                         Qdrant Cloud                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Collection: book_chunks                                                │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ Point                                                            │   │
│  │─────────────────────────────────────────────────────────────────│   │
│  │ id: UUID                                                         │   │
│  │ vector: float[1536]  (text-embedding-3-small)                   │   │
│  │ payload:                                                         │   │
│  │   - chapter: string                                              │   │
│  │   - source: string                                               │   │
│  │   - location: string                                             │   │
│  │   - page_id: string                                              │   │
│  │   - text: string                                                 │   │
│  │   - token_count: number                                          │   │
│  │   - created_at: timestamp                                        │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## PostgreSQL Entities

### 1. Users (Managed by Better Auth)

**Table**: `users`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PK, DEFAULT gen_random_uuid() | Unique identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL | User email address |
| name | VARCHAR(255) | NULL | Display name |
| password | VARCHAR(255) | NOT NULL | Bcrypt hashed password |
| role | VARCHAR(50) | NOT NULL, DEFAULT 'user' | 'user' or 'admin' |
| email_verified | BOOLEAN | NOT NULL, DEFAULT false | Email verification status |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Account creation time |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update time |

**Indexes**:
- `idx_users_email` on `(email)`

**Validation Rules**:
- Email must be valid format
- Password minimum 8 characters (enforced at application layer)
- Role must be 'user' or 'admin'

---

### 2. Sessions (Managed by Better Auth)

**Table**: `sessions`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PK, DEFAULT gen_random_uuid() | Session identifier |
| user_id | UUID | FK → users(id) ON DELETE CASCADE | Session owner |
| token | VARCHAR(255) | UNIQUE, NOT NULL | Session token |
| expires_at | TIMESTAMP | NOT NULL | Token expiration |
| device_info | JSONB | NULL | Device metadata |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Session start |

**Indexes**:
- `idx_sessions_user_id` on `(user_id)`
- `idx_sessions_token` on `(token)`
- `idx_sessions_expires_at` on `(expires_at)`

---

### 3. Conversations

**Table**: `conversations`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PK, DEFAULT gen_random_uuid() | Conversation identifier |
| user_id | UUID | FK → users(id) ON DELETE CASCADE, NOT NULL | Conversation owner |
| title | VARCHAR(255) | NULL | Auto-generated or user-set title |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Conversation start |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last activity |

**Indexes**:
- `idx_conversations_user_id_updated` on `(user_id, updated_at DESC)`

---

### 4. Messages

**Table**: `messages`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PK, DEFAULT gen_random_uuid() | Message identifier |
| conversation_id | UUID | FK → conversations(id) ON DELETE CASCADE, NOT NULL | Parent conversation |
| role | VARCHAR(50) | NOT NULL | 'user' or 'assistant' |
| content | TEXT | NOT NULL | Message text |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Message timestamp |

**Indexes**:
- `idx_messages_conversation_created` on `(conversation_id, created_at DESC, id DESC)` — For cursor pagination

**Validation Rules**:
- Role must be 'user' or 'assistant'
- Content cannot be empty

---

### 5. Rate Limits

**Table**: `rate_limits`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| user_id | UUID | PK, FK → users(id) ON DELETE CASCADE | User identifier |
| tokens | DECIMAL(10,4) | NOT NULL, DEFAULT 10.0 | Available tokens (max 10) |
| last_refill | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last token refill time |

**Algorithm**: Token bucket with refill rate of 10 tokens per 60 seconds (0.1667/sec)

---

## Qdrant Collection

### Collection: book_chunks

**Configuration**:
```json
{
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "optimizers_config": {
    "memmap_threshold": 20000
  }
}
```

**Point Structure**:

| Field | Type | Description |
|-------|------|-------------|
| id | UUID | Unique chunk identifier |
| vector | float[1536] | Embedding from text-embedding-3-small |
| payload.chapter | string | Chapter name (e.g., "Chapter 2: ROS 2") |
| payload.source | string | Source file or section |
| payload.location | string | Location within chapter |
| payload.page_id | string | Reference to book page |
| payload.text | string | Original chunk text (400-700 tokens) |
| payload.token_count | number | Token count for context budgeting |
| payload.created_at | string | ISO timestamp of ingestion |

**Payload Indexes**:
- `chapter` — For filtering by chapter
- `page_id` — For filtering by page

---

## Drizzle ORM Schema

```typescript
// src/db/schema.ts
import { pgTable, uuid, varchar, text, timestamp, boolean, decimal, jsonb, index } from "drizzle-orm/pg-core";

export const users = pgTable("users", {
  id: uuid("id").primaryKey().defaultRandom(),
  email: varchar("email", { length: 255 }).unique().notNull(),
  name: varchar("name", { length: 255 }),
  password: varchar("password", { length: 255 }).notNull(),
  role: varchar("role", { length: 50 }).notNull().default("user"),
  emailVerified: boolean("email_verified").notNull().default(false),
  createdAt: timestamp("created_at").notNull().defaultNow(),
  updatedAt: timestamp("updated_at").notNull().defaultNow(),
}, (table) => [
  index("idx_users_email").on(table.email),
]);

export const sessions = pgTable("sessions", {
  id: uuid("id").primaryKey().defaultRandom(),
  userId: uuid("user_id").notNull().references(() => users.id, { onDelete: "cascade" }),
  token: varchar("token", { length: 255 }).unique().notNull(),
  expiresAt: timestamp("expires_at").notNull(),
  deviceInfo: jsonb("device_info"),
  createdAt: timestamp("created_at").notNull().defaultNow(),
}, (table) => [
  index("idx_sessions_user_id").on(table.userId),
  index("idx_sessions_token").on(table.token),
]);

export const conversations = pgTable("conversations", {
  id: uuid("id").primaryKey().defaultRandom(),
  userId: uuid("user_id").notNull().references(() => users.id, { onDelete: "cascade" }),
  title: varchar("title", { length: 255 }),
  createdAt: timestamp("created_at").notNull().defaultNow(),
  updatedAt: timestamp("updated_at").notNull().defaultNow(),
}, (table) => [
  index("idx_conversations_user_updated").on(table.userId, table.updatedAt),
]);

export const messages = pgTable("messages", {
  id: uuid("id").primaryKey().defaultRandom(),
  conversationId: uuid("conversation_id").notNull().references(() => conversations.id, { onDelete: "cascade" }),
  role: varchar("role", { length: 50 }).notNull(), // 'user' | 'assistant'
  content: text("content").notNull(),
  createdAt: timestamp("created_at").notNull().defaultNow(),
}, (table) => [
  index("idx_messages_conversation_created").on(table.conversationId, table.createdAt),
]);

export const rateLimits = pgTable("rate_limits", {
  userId: uuid("user_id").primaryKey().references(() => users.id, { onDelete: "cascade" }),
  tokens: decimal("tokens", { precision: 10, scale: 4 }).notNull().default("10.0"),
  lastRefill: timestamp("last_refill").notNull().defaultNow(),
});
```

---

## State Transitions

### User Verification Flow

```
[Unregistered]
    │ POST /api/auth/signup
    ▼
[Unverified] ─── email_verified: false
    │ Click verification link
    ▼
[Verified] ─── email_verified: true
    │ Can access chatbot
    ▼
[Active User]
```

### Rate Limit State

```
[Full] tokens = 10.0
    │ User sends message
    ▼
[Available] tokens = 9.0 ... 1.0
    │ User sends message
    ▼
[Exhausted] tokens < 1.0
    │ Wait ~6 seconds per token
    ▼
[Refilling] tokens += 0.1667/sec
    │ tokens >= 1.0
    ▼
[Available]
```

---

## Query Patterns

### 1. Get Conversation History (Cursor Pagination)

```typescript
// Most recent messages first, cursor-based
const getHistory = await db
  .select()
  .from(messages)
  .where(
    and(
      eq(messages.conversationId, conversationId),
      cursor ? lt(messages.createdAt, cursor.createdAt) : undefined
    )
  )
  .orderBy(desc(messages.createdAt), desc(messages.id))
  .limit(21); // +1 to detect hasMore
```

### 2. Vector Search with Threshold

```typescript
const results = await qdrant.search("book_chunks", {
  vector: queryEmbedding,
  limit: 5,
  score_threshold: 0.7,
  with_payload: true,
  filter: {
    must: [
      { key: "chapter", match: { value: chapterFilter } } // Optional
    ]
  }
});
```

### 3. Check Rate Limit

```typescript
const result = await db.execute(sql`SELECT check_rate_limit(${userId}::uuid)`);
const allowed = result.rows[0].check_rate_limit;
```

---

## Data Retention

| Entity | Retention | Archive Strategy |
|--------|-----------|------------------|
| Users | Indefinite | N/A |
| Sessions | Auto-cleanup on expiry | Delete expired sessions daily |
| Conversations | 1 year | Archive to cold storage |
| Messages | 1 year | Archive with parent conversation |
| Rate Limits | Active users only | Delete on user deletion |
| Book Chunks | Indefinite | Version with updates |
