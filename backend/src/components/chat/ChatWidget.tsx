"use client";

import { ChatHistory } from "./ChatHistory";
import { ChatInput } from "./ChatInput";
import { useChat } from "@/hooks/useChat";

interface ChatWidgetProps {
  conversationId?: string;
}

export function ChatWidget({ conversationId: initialConversationId }: ChatWidgetProps) {
  const {
    messages,
    streamingContent,
    isLoading,
    error,
    sendMessage,
    conversationId,
  } = useChat(initialConversationId);

  return (
    <div className="flex flex-col h-full bg-white rounded-lg shadow-lg overflow-hidden">
      <div className="bg-blue-600 text-white px-4 py-3 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <span className="text-xl">📚</span>
          <h2 className="font-semibold">Book Assistant</h2>
        </div>
        {conversationId && (
          <span className="text-xs text-blue-200">
            Conversation active
          </span>
        )}
      </div>

      <ChatHistory messages={messages} streamingContent={streamingContent} />

      {error && (
        <div className="px-4 py-2 bg-red-50 text-red-700 text-sm">
          {error}
        </div>
      )}

      <ChatInput onSend={sendMessage} disabled={isLoading} />
    </div>
  );
}
