import { NextRequest } from "next/server";
import {
  requireVerifiedEmail,
  errorToResponse,
  createApiError,
} from "@/lib/auth-utils";
import { checkRateLimit } from "@/lib/rate-limit";
import { retrieveContext } from "@/lib/rag/retrieve";
import { streamChatResponse, generateTitle } from "@/lib/gemini";
import { db, conversations, messages } from "@/lib/db";
import { eq } from "drizzle-orm";
import { z } from "zod";
import { v4 as uuidv4 } from "uuid";

const contextChatSchema = z.object({
  message: z.string().min(1, "Message is required").max(2000, "Message too long"),
  selectedText: z.string().min(1, "Selected text is required").max(5000, "Selected text too long"),
  pageId: z.string().optional(),
  conversationId: z.string().uuid().optional(),
});

export async function POST(request: NextRequest) {
  try {
    // Require verified email
    const { user } = await requireVerifiedEmail();

    // Check rate limit
    const rateLimitResult = await checkRateLimit(user.id);
    if (!rateLimitResult.allowed) {
      throw createApiError(
        "RATE_LIMITED",
        `Rate limit exceeded. Try again in ${rateLimitResult.retryAfter} seconds.`,
        rateLimitResult.retryAfter
      );
    }

    // Parse and validate body
    const body = await request.json();
    const parsed = contextChatSchema.safeParse(body);

    if (!parsed.success) {
      throw createApiError("BAD_REQUEST", parsed.error.issues[0].message);
    }

    const { message, selectedText, conversationId: existingConversationId } = parsed.data;

    // Use selected text as additional context
    const { contextText: ragContext } = await retrieveContext(message);

    // Combine selected text with RAG context
    const combinedContext = `
Selected text from the page:
---
${selectedText}
---

Additional context from the book:
---
${ragContext}
---
`.trim();

    // Get or create conversation
    let conversationId = existingConversationId;
    if (!conversationId) {
      const title = await generateTitle(message);
      const [newConversation] = await db
        .insert(conversations)
        .values({
          userId: user.id,
          title,
        })
        .returning();
      conversationId = newConversation.id;
    }

    // Save user message with context indicator
    await db.insert(messages).values({
      conversationId,
      role: "user",
      content: `[Context from page] ${message}`,
    });

    // Create SSE response
    const encoder = new TextEncoder();
    const stream = new ReadableStream({
      async start(controller) {
        const messageId = uuidv4();
        let fullResponse = "";

        try {
          // Stream response from Gemini with combined context
          for await (const token of streamChatResponse(combinedContext, message)) {
            fullResponse += token;
            controller.enqueue(
              encoder.encode(
                `data: ${JSON.stringify({ type: "token", data: token })}\n\n`
              )
            );
          }

          // Save assistant message
          await db.insert(messages).values({
            id: messageId,
            conversationId,
            role: "assistant",
            content: fullResponse,
          });

          // Update conversation timestamp
          await db
            .update(conversations)
            .set({ updatedAt: new Date() })
            .where(eq(conversations.id, conversationId));

          // Send done event
          controller.enqueue(
            encoder.encode(
              `data: ${JSON.stringify({
                type: "done",
                conversationId,
                messageId,
              })}\n\n`
            )
          );
        } catch (error) {
          console.error("Streaming error:", error);
          controller.enqueue(
            encoder.encode(
              `data: ${JSON.stringify({
                type: "error",
                error: "INTERNAL_ERROR",
                message: "Failed to generate response",
              })}\n\n`
            )
          );
        } finally {
          controller.close();
        }
      },
    });

    return new Response(stream, {
      headers: {
        "Content-Type": "text/event-stream",
        "Cache-Control": "no-cache",
        Connection: "keep-alive",
      },
    });
  } catch (error) {
    return errorToResponse(error);
  }
}
