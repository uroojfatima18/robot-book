"use client";

import { useRef, useEffect } from "react";
import { ChatMessage } from "./ChatMessage";
import { Message } from "@/types";

interface ChatHistoryProps {
  messages: Message[];
  streamingContent?: string;
}

export function ChatHistory({ messages, streamingContent }: ChatHistoryProps) {
  const bottomRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages, streamingContent]);

  if (messages.length === 0 && !streamingContent) {
    return (
      <div className="flex-1 flex items-center justify-center p-8">
        <div className="text-center max-w-md">
          <div className="text-6xl mb-4 text-primary-600">📚</div>
          <h2 className="text-xl font-semibold text-gray-900 mb-2">
            Ask about the Robotics Book
          </h2>
          <p className="text-gray-600 mb-4">
            I can answer questions about ROS 2, digital twins, AI-powered robots,
            and other topics covered in the book.
          </p>
          <div className="text-sm text-gray-500 space-y-1">
            <p>Try asking:</p>
            <p className="italic text-gray-700">&quot;How do ROS 2 nodes communicate?&quot;</p>
            <p className="italic text-gray-700">&quot;What is a digital twin?&quot;</p>
            <p className="italic text-gray-700">&quot;Explain the robot&apos;s nervous system&quot;</p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="flex-1 overflow-y-auto p-4">
      {messages.map((message) => (
        <ChatMessage
          key={message.id}
          role={message.role}
          content={message.content}
        />
      ))}
      {streamingContent && (
        <ChatMessage
          role="assistant"
          content={streamingContent}
          isStreaming={true}
        />
      )}
      <div ref={bottomRef} />
    </div>
  );
}
