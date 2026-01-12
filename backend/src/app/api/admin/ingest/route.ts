import { NextRequest } from "next/server";
import { requireAdmin, errorToResponse, createApiError } from "@/lib/auth-utils";
import { ingestContent } from "@/lib/rag/ingest";
import { z } from "zod";
import { v4 as uuidv4 } from "uuid";

const ingestSchema = z.object({
  content: z.string().min(1, "Content is required"),
  chapter: z.string().min(1, "Chapter name is required"),
  source: z.string().optional(),
  pageId: z.string().optional(),
});

export async function POST(request: NextRequest) {
  try {
    // Require admin auth
    await requireAdmin();

    // Parse and validate body
    const body = await request.json();
    const parsed = ingestSchema.safeParse(body);

    if (!parsed.success) {
      throw createApiError("BAD_REQUEST", parsed.error.issues[0].message);
    }

    const { content, chapter, source, pageId } = parsed.data;

    // Ingest content
    const result = await ingestContent(content, {
      chapter,
      source,
      pageId,
    });

    return Response.json({
      jobId: uuidv4(),
      chunksCreated: result.chunksCreated,
      totalTokens: result.totalTokens,
      status: "completed",
    });
  } catch (error) {
    return errorToResponse(error);
  }
}
