"use client";

import { useState, useEffect, useCallback } from "react";
import { ConversationCard } from "./ConversationCard";
import { ConversationSummary } from "@/types";

interface ConversationListProps {
  currentConversationId?: string;
}

export function ConversationList({ currentConversationId }: ConversationListProps = {}) {
  const [conversations, setConversations] = useState<ConversationSummary[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [hasMore, setHasMore] = useState(false);
  const [cursor, setCursor] = useState<string | undefined>();

  const fetchConversations = useCallback(async (nextCursor?: string) => {
    try {
      setIsLoading(true);
      const params = new URLSearchParams();
      if (nextCursor) params.set("cursor", nextCursor);

      const response = await fetch(`/api/conversations?${params}`);

      if (!response.ok) {
        throw new Error("Failed to fetch conversations");
      }

      const data = await response.json();

      if (nextCursor) {
        setConversations((prev) => [...prev, ...data.conversations]);
      } else {
        setConversations(data.conversations);
      }

      setHasMore(data.hasMore);
      setCursor(data.nextCursor);
    } catch (err) {
      setError(err instanceof Error ? err.message : "An error occurred");
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchConversations();
  }, [fetchConversations]);

  const handleDelete = async (id: string) => {
    if (!confirm("Are you sure you want to delete this conversation?")) {
      return;
    }

    try {
      const response = await fetch(`/api/conversations/${id}`, {
        method: "DELETE",
      });

      if (!response.ok) {
        throw new Error("Failed to delete conversation");
      }

      setConversations((prev) => prev.filter((c) => c.id !== id));
    } catch {
      alert("Failed to delete conversation");
    }
  };

  if (error) {
    return (
      <div className="text-center py-8">
        <p className="text-red-600">{error}</p>
        <button
          onClick={() => fetchConversations()}
          className="mt-4 text-blue-600 hover:text-blue-700"
        >
          Try again
        </button>
      </div>
    );
  }

  if (isLoading && conversations.length === 0) {
    return (
      <div className="space-y-4">
        {[1, 2, 3].map((i) => (
          <div
            key={i}
            className="bg-white rounded-lg border border-gray-200 p-4 animate-pulse"
          >
            <div className="h-5 bg-gray-200 rounded w-3/4 mb-2"></div>
            <div className="h-4 bg-gray-200 rounded w-1/4"></div>
          </div>
        ))}
      </div>
    );
  }

  if (conversations.length === 0) {
    return (
      <div className="text-center py-12">
        <div className="text-6xl mb-4">💬</div>
        <h3 className="text-lg font-medium text-gray-900 mb-2">
          No conversations yet
        </h3>
        <p className="text-gray-500">
          Start chatting with the book assistant to see your history here.
        </p>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      {conversations.map((conversation) => (
        <ConversationCard
          key={conversation.id}
          conversation={conversation}
          onDelete={handleDelete}
          isActive={conversation.id === currentConversationId}
        />
      ))}

      {hasMore && (
        <div className="text-center">
          <button
            onClick={() => fetchConversations(cursor)}
            disabled={isLoading}
            className="text-blue-600 hover:text-blue-700 disabled:opacity-50"
          >
            {isLoading ? "Loading..." : "Load more"}
          </button>
        </div>
      )}
    </div>
  );
}
