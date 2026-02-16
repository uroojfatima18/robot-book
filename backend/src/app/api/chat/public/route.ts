import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";

const chatSchema = z.object({
  message: z.string().min(1, "Message is required").max(2000, "Message too long"),
});

// Rate limiting
const rateLimitMap = new Map<string, { count: number; resetTime: number }>();
const RATE_LIMIT = 20;
const RATE_WINDOW = 60 * 1000;

function checkRateLimit(ip: string): boolean {
  const now = Date.now();
  const record = rateLimitMap.get(ip);
  if (!record || now > record.resetTime) {
    rateLimitMap.set(ip, { count: 1, resetTime: now + RATE_WINDOW });
    return true;
  }
  if (record.count >= RATE_LIMIT) return false;
  record.count++;
  return true;
}

export async function POST(request: NextRequest) {
  try {
    const ip = request.headers.get("x-forwarded-for") || "anonymous";
    if (!checkRateLimit(ip)) {
      return NextResponse.json({ error: "Rate limit exceeded" }, { status: 429 });
    }

    let body;
    try {
      body = await request.json();
    } catch {
      return NextResponse.json({ error: "Invalid JSON" }, { status: 400 });
    }

    const parsed = chatSchema.safeParse(body);
    if (!parsed.success) {
      return NextResponse.json({ error: parsed.error.issues[0].message }, { status: 400 });
    }

    const { message } = parsed.data;

    // Check if Gemini API key is configured
    if (!process.env.GEMINI_API_KEY) {
      return NextResponse.json({ error: "GEMINI_API_KEY not configured" }, { status: 500 });
    }

    // Import RAG and Gemini modules
    const { retrieveContext } = await import("@/lib/rag/retrieve");
    const { streamChatResponse } = await import("@/lib/gemini");

    const encoder = new TextEncoder();
    const stream = new ReadableStream({
      async start(controller) {
        try {
          // Retrieve relevant context from vector DB
          const { contextText, chunks } = await retrieveContext(message);

          if (chunks.length === 0) {
            const noContentMsg = "I couldn't find relevant information about that in the book. Please ask about ROS 2, digital twins, AI robotics, navigation, SLAM, or other topics covered in the robotics chapters.";
            controller.enqueue(encoder.encode(`data: ${JSON.stringify({ type: "token", data: noContentMsg })}\n\n`));
            controller.enqueue(encoder.encode(`data: ${JSON.stringify({ type: "done" })}\n\n`));
            controller.close();
            return;
          }

          // Stream response from Gemini
          try {
            for await (const token of streamChatResponse(contextText, message)) {
              controller.enqueue(encoder.encode(`data: ${JSON.stringify({ type: "token", data: token })}\n\n`));
            }
          } catch (geminiError: any) {
            // Handle Gemini API errors (quota exceeded, etc.)
            if (geminiError.message?.includes('429') || geminiError.message?.includes('quota')) {
              const quotaMsg = "⚠️ The AI service is currently at capacity. This is a temporary issue with the Gemini API quota. Please try again in a few minutes, or contact the administrator to update the API key.";
              controller.enqueue(encoder.encode(`data: ${JSON.stringify({ type: "token", data: quotaMsg })}\n\n`));
            } else {
              throw geminiError;
            }
          }

          controller.enqueue(encoder.encode(`data: ${JSON.stringify({ type: "done" })}\n\n`));
        } catch (error) {
          console.error("Streaming error:", error);
          const errMsg = error instanceof Error ? error.message : "Unknown error";
          controller.enqueue(encoder.encode(`data: ${JSON.stringify({ type: "token", data: `Error: ${errMsg}` })}\n\n`));
          controller.enqueue(encoder.encode(`data: ${JSON.stringify({ type: "done" })}\n\n`));
        } finally {
          controller.close();
        }
      },
    });

    return new Response(stream, {
      headers: {
        "Content-Type": "text/event-stream",
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
      },
    });
  } catch (error) {
    console.error("Chat error:", error);
    return NextResponse.json({ error: "Internal server error" }, { status: 500 });
  }
}

export async function OPTIONS() {
  return new NextResponse(null, {
    status: 200,
    headers: {
      "Access-Control-Allow-Origin": "http://localhost:3005",
      "Access-Control-Allow-Methods": "POST, OPTIONS",
      "Access-Control-Allow-Headers": "Content-Type",
    },
  });
}
