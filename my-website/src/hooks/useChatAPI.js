"use client";

import { useState, useCallback } from "react";

const API_BASE_URL = "http://localhost:3001";

export function useChatAPI() {
  const [messages, setMessages] = useState([]);
  const [streamingContent, setStreamingContent] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const sendMessage = useCallback(async (content) => {
    setError(null);
    setIsLoading(true);
    setStreamingContent("");

    // Add user message immediately
    const userMessage = {
      id: Date.now(),
      role: "user",
      content,
      createdAt: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);

    try {
      const response = await fetch(`${API_BASE_URL}/api/chat/public`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ message: content }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.error || "Failed to send message");
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
              const event = JSON.parse(data);

              if (event.type === "token") {
                fullContent += event.data;
                setStreamingContent(fullContent);
              } else if (event.type === "done") {
                // Add assistant message
                const assistantMessage = {
                  id: Date.now() + 1,
                  role: "assistant",
                  content: fullContent,
                  createdAt: new Date(),
                };
                setMessages((prev) => [...prev, assistantMessage]);
                setStreamingContent("");
              } else if (event.type === "error") {
                throw new Error(event.message || "Error from server");
              }
            } catch (parseError) {
              // Ignore JSON parse errors for partial chunks
              console.debug("Parse error (expected for partial chunks):", parseError);
            }
          }
        }
      }
    } catch (err) {
      console.error("Chat error:", err);
      setError(err.message || "An error occurred");
      // Remove the user message if there was an error
      setMessages((prev) => prev.filter((m) => m.id !== userMessage.id));
    } finally {
      setIsLoading(false);
      setStreamingContent("");
    }
  }, []);

  const clearMessages = useCallback(() => {
    setMessages([]);
    setError(null);
  }, []);

  return {
    messages,
    streamingContent,
    isLoading,
    error,
    sendMessage,
    clearMessages,
  };
}
