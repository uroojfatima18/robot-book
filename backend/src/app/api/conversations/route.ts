import { NextRequest } from "next/server";
import { requireVerifiedEmail, errorToResponse } from "@/lib/auth-utils";
import { db, conversations, messages } from "@/lib/db";
import { eq, desc, lt, count, and } from "drizzle-orm";

const PAGE_SIZE = 20;

export async function GET(request: NextRequest) {
  try {
    const { user } = await requireVerifiedEmail();

    const searchParams = request.nextUrl.searchParams;
    const cursor = searchParams.get("cursor");

    // Build query
    const conditions = [eq(conversations.userId, user.id)];
    if (cursor) {
      conditions.push(lt(conversations.updatedAt, new Date(cursor)));
    }

    // Fetch conversations with message count
    const result = await db
      .select({
        id: conversations.id,
        title: conversations.title,
        createdAt: conversations.createdAt,
        updatedAt: conversations.updatedAt,
      })
      .from(conversations)
      .where(and(...conditions))
      .orderBy(desc(conversations.updatedAt))
      .limit(PAGE_SIZE + 1);

    // Get message counts for each conversation
    const conversationIds = result.slice(0, PAGE_SIZE).map((c) => c.id);
    const messageCounts = await Promise.all(
      conversationIds.map(async (id) => {
        const [{ count: msgCount }] = await db
          .select({ count: count() })
          .from(messages)
          .where(eq(messages.conversationId, id));
        return { id, count: msgCount };
      })
    );

    const countMap = new Map(messageCounts.map((m) => [m.id, m.count]));

    const hasMore = result.length > PAGE_SIZE;
    const items = result.slice(0, PAGE_SIZE).map((c) => ({
      ...c,
      messageCount: countMap.get(c.id) || 0,
    }));

    return Response.json({
      conversations: items,
      hasMore,
      nextCursor: hasMore ? items[items.length - 1].updatedAt.toISOString() : undefined,
    });
  } catch (error) {
    return errorToResponse(error);
  }
}
