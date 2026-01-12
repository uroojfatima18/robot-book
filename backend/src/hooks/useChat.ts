"use client";

import { useState, useCallback } from "react";
import { Message, SSEEvent } from "@/types";
import { v4 as uuidv4 } from "uuid";

export function useChat(initialConversationId?: string) {
  const [messages, setMessages] = useState<Message[]>([]);
  const [streamingContent, setStreamingContent] = useState<string>("");
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [conversationId, setConversationId] = useState<string | undefined>(
    initialConversationId
  );

  const sendMessage = useCallback(
    async (content: string) => {
      setError(null);
      setIsLoading(true);
      setStreamingContent("");

      // Add user message immediately
      const userMessage: Message = {
        id: uuidv4(),
        role: "user",
        content,
        createdAt: new Date(),
      };
      setMessages((prev) => [...prev, userMessage]);

      try {
        const response = await fetch("/api/chat/public", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            message: content,
          }),
        });

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.message || "Failed to send message");
        }

        if (!response.body) {
          throw new Error("No response body");
        }

        // Read SSE stream
        const reader = response.body.getReader();
        const decoder = new TextDecoder();
        let fullContent = "";

        while (true) {
          const { done, value } = await reader.read();
          if (done) break;

          const chunk = decoder.decode(value);
          const lines = chunk.split("\n");

          for (const line of lines) {
            if (line.startsWith("data: ")) {
              const data = line.slice(6);
              if (data === "[DONE]") continue;

              try {
                const event: SSEEvent = JSON.parse(data);

                if (event.type === "token") {
                  fullContent += event.data;
                  setStreamingContent(fullContent);
                } else if (event.type === "done") {
                  setConversationId(event.conversationId);
                  // Add assistant message
                  const assistantMessage: Message = {
                    id: event.messageId,
                    role: "assistant",
                    content: fullContent,
                    createdAt: new Date(),
                  };
                  setMessages((prev) => [...prev, assistantMessage]);
                  setStreamingContent("");
                } else if (event.type === "error") {
                  throw new Error(event.message);
                }
              } catch {
                // Ignore JSON parse errors for partial chunks
              }
            }
          }
        }
      } catch (err) {
        setError(err instanceof Error ? err.message : "An error occurred");
        // Remove the user message if there was an error
        setMessages((prev) => prev.filter((m) => m.id !== userMessage.id));
      } finally {
        setIsLoading(false);
        setStreamingContent("");
      }
    },
    [conversationId]
  );

  const clearMessages = useCallback(() => {
    setMessages([]);
    setConversationId(undefined);
    setError(null);
  }, []);

  return {
    messages,
    streamingContent,
    isLoading,
    error,
    conversationId,
    sendMessage,
    clearMessages,
  };
}
