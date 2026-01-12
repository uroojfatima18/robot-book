import { NextRequest } from "next/server";
import {
  requireVerifiedEmail,
  errorToResponse,
  createApiError,
} from "@/lib/auth-utils";
import { db, conversations, messages } from "@/lib/db";
import { eq, and, desc, lt } from "drizzle-orm";

const PAGE_SIZE = 50;

export async function GET(
  request: NextRequest,
  { params }: { params: { id: string } }
) {
  try {
    const { user } = await requireVerifiedEmail();
    const { id } = params;

    // Verify ownership
    const [conversation] = await db
      .select()
      .from(conversations)
      .where(and(eq(conversations.id, id), eq(conversations.userId, user.id)))
      .limit(1);

    if (!conversation) {
      throw createApiError("NOT_FOUND", "Conversation not found");
    }

    const searchParams = request.nextUrl.searchParams;
    const cursor = searchParams.get("cursor");

    // Build query for messages
    const conditions = [eq(messages.conversationId, id)];
    if (cursor) {
      conditions.push(lt(messages.createdAt, new Date(cursor)));
    }

    const result = await db
      .select()
      .from(messages)
      .where(and(...conditions))
      .orderBy(desc(messages.createdAt))
      .limit(PAGE_SIZE + 1);

    const hasMore = result.length > PAGE_SIZE;
    const items = result.slice(0, PAGE_SIZE).reverse(); // Reverse to show oldest first

    return Response.json({
      id: conversation.id,
      title: conversation.title,
      messages: items.map((m) => ({
        id: m.id,
        role: m.role,
        content: m.content,
        createdAt: m.createdAt,
      })),
      hasMore,
      nextCursor: hasMore ? result[PAGE_SIZE].createdAt.toISOString() : undefined,
      createdAt: conversation.createdAt,
    });
  } catch (error) {
    return errorToResponse(error);
  }
}

export async function DELETE(
  request: NextRequest,
  { params }: { params: { id: string } }
) {
  try {
    const { user } = await requireVerifiedEmail();
    const { id } = params;

    // Verify ownership and delete
    const result = await db
      .delete(conversations)
      .where(and(eq(conversations.id, id), eq(conversations.userId, user.id)))
      .returning();

    if (result.length === 0) {
      throw createApiError("NOT_FOUND", "Conversation not found");
    }

    return Response.json({ success: true });
  } catch (error) {
    return errorToResponse(error);
  }
}
