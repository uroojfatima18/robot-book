import { requireAdmin, errorToResponse } from "@/lib/auth-utils";
import { getCollectionStats } from "@/lib/qdrant";
import { db, messages, users, conversations } from "@/lib/db";
import { count } from "drizzle-orm";

export async function GET() {
  try {
    await requireAdmin();

    // Get Qdrant stats
    const qdrantStats = await getCollectionStats();

    // Get database stats
    const [userCount] = await db.select({ count: count() }).from(users);
    const [conversationCount] = await db.select({ count: count() }).from(conversations);
    const [messageCount] = await db.select({ count: count() }).from(messages);

    return Response.json({
      vectorDatabase: {
        vectorsCount: qdrantStats.vectorsCount,
        pointsCount: qdrantStats.pointsCount,
      },
      database: {
        users: userCount.count,
        conversations: conversationCount.count,
        messages: messageCount.count,
      },
    });
  } catch (error) {
    return errorToResponse(error);
  }
}
