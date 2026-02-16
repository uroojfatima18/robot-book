/**
 * API Types for RAG Chatbot with Authentication
 */

// ============================================================
// User & Auth Types (Better Auth managed)
// ============================================================

export type UserRole = "user" | "admin";

export interface User {
  id: string;
  email: string;
  name: string | null;
  role: UserRole;
  emailVerified: boolean;
  createdAt: Date;
  updatedAt: Date;
}

export interface Session {
  id: string;
  userId: string;
  token: string;
  expiresAt: Date;
  deviceInfo?: Record<string, unknown>;
  createdAt: Date;
}

// ============================================================
// Chat Types
// ============================================================

export interface ChatRequest {
  message: string;
  conversationId?: string;
}

export interface ContextualChatRequest {
  message: string;
  selectedText: string;
  pageId?: string;
  conversationId?: string;
}

export type MessageRole = "user" | "assistant";

export interface Message {
  id: string;
  role: MessageRole;
  content: string;
  createdAt: Date;
}

// ============================================================
// Conversation Types
// ============================================================

export interface ConversationSummary {
  id: string;
  title: string | null;
  messageCount: number;
  createdAt: Date;
  updatedAt: Date;
}

export interface ConversationDetail {
  id: string;
  title: string | null;
  messages: Message[];
  hasMore: boolean;
  nextCursor?: string;
  createdAt: Date;
}

export interface ConversationListResponse {
  conversations: ConversationSummary[];
  hasMore: boolean;
  nextCursor?: string;
}

// ============================================================
// Admin Types
// ============================================================

export interface IngestRequest {
  content: string;
  chapter: string;
  source?: string;
  pageId?: string;
}

export type IngestStatus = "processing" | "completed" | "failed";

export interface IngestResponse {
  jobId: string;
  chunksCreated: number;
  status: IngestStatus;
}

export type StatsPeriod = "day" | "week" | "month";

export interface ChapterStats {
  chapter: string;
  queryCount: number;
}

export interface UsageStats {
  period: StatsPeriod;
  totalMessages: number;
  uniqueUsers: number;
  avgResponseTime: number;
  topChapters: ChapterStats[];
}

// ============================================================
// Error Types
// ============================================================

export type ErrorCode =
  | "UNAUTHORIZED"
  | "EMAIL_NOT_VERIFIED"
  | "FORBIDDEN"
  | "RATE_LIMITED"
  | "NOT_FOUND"
  | "BAD_REQUEST"
  | "INTERNAL_ERROR";

export interface ApiError {
  error: ErrorCode;
  message: string;
  retryAfter?: number;
}

// ============================================================
// Vector/RAG Types (Internal)
// ============================================================

export interface ChunkPayload {
  chapter: string;
  source: string;
  location: string;
  pageId: string;
  text: string;
  tokenCount: number;
  createdAt: string;
}

export interface ScoredChunk {
  id: string;
  score: number;
  payload: ChunkPayload;
}

export interface RetrievalResult {
  chunks: ScoredChunk[];
  totalTokens: number;
  contextText: string;
}

// ============================================================
// SSE Event Types (Streaming)
// ============================================================

export type SSEEventType = "token" | "done" | "error";

export interface SSETokenEvent {
  type: "token";
  data: string;
}

export interface SSEDoneEvent {
  type: "done";
  conversationId: string;
  messageId: string;
}

export interface SSEErrorEvent {
  type: "error";
  error: ErrorCode;
  message: string;
}

export type SSEEvent = SSETokenEvent | SSEDoneEvent | SSEErrorEvent;

// ============================================================
// Request/Response Helpers
// ============================================================

export interface PaginationParams {
  limit?: number;
  cursor?: string;
}

export interface PaginatedResponse<T> {
  items: T[];
  hasMore: boolean;
  nextCursor?: string;
}
