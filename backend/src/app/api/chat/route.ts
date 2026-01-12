import { NextRequest } from "next/server";
import {
  requireVerifiedEmail,
  errorToResponse,
  createApiError,
} from "@/lib/auth-utils";
import { checkRateLimit } from "@/lib/rate-limit";
import { retrieveContext, hasRelevantContent } from "@/lib/rag/retrieve";
import { streamChatResponse, generateTitle } from "@/lib/gemini";
import { db, conversations, messages } from "@/lib/db";
import { eq } from "drizzle-orm";
import { z } from "zod";
import { v4 as uuidv4 } from "uuid";

const chatSchema = z.object({
  message: z.string().min(1, "Message is required").max(2000, "Message too long"),
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
    const parsed = chatSchema.safeParse(body);

    if (!parsed.success) {
      throw createApiError("BAD_REQUEST", parsed.error.issues[0].message);
    }

    const { message, conversationId: existingConversationId } = parsed.data;

    // Check if we have relevant content
    const hasContent = await hasRelevantContent(message);

    // Retrieve context from vector database
    const { contextText, chunks } = await retrieveContext(message);

    // Get or create conversation
    let conversationId = existingConversationId;
    if (!conversationId) {
      // Create new conversation
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

    // Save user message
    await db.insert(messages).values({
      conversationId,
      role: "user",
      content: message,
    });

    // Create SSE response
    const encoder = new TextEncoder();
    const stream = new ReadableStream({
      async start(controller) {
        const messageId = uuidv4();
        let fullResponse = "";

        try {
          // Check if we have content to answer from
          if (!hasContent || chunks.length === 0) {
            const noContentResponse =
              "I couldn't find relevant information about that in the book content. Please try asking about topics covered in the robotics chapters, such as ROS 2, digital twins, or AI-powered robots.";

            // Send as single token
            controller.enqueue(
              encoder.encode(
                `data: ${JSON.stringify({ type: "token", data: noContentResponse })}\n\n`
              )
            );
            fullResponse = noContentResponse;
          } else {
            // Stream response from Gemini
            for await (const token of streamChatResponse(contextText, message)) {
              fullResponse += token;
              controller.enqueue(
                encoder.encode(
                  `data: ${JSON.stringify({ type: "token", data: token })}\n\n`
                )
              );
            }
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
