"use client";

import Link from "next/link";
import { ConversationSummary } from "@/types";

interface ConversationCardProps {
  conversation: ConversationSummary;
  onDelete?: (id: string) => void;
  isActive?: boolean;
}

export function ConversationCard({ conversation, onDelete, isActive = false }: ConversationCardProps) {
  const formattedDate = new Date(conversation.updatedAt).toLocaleDateString(
    undefined,
    {
      month: "short",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    }
  );

  return (
    <div className={`rounded-lg p-4 hover:shadow-md transition-shadow ${
      isActive
        ? 'bg-blue-50 border border-blue-200'
        : 'bg-white border border-gray-200 hover:border-gray-300'
    }`}>
      <div className="flex justify-between items-start">
        <Link href={`/chat?id=${conversation.id}`} className="flex-1">
          <h3 className={`font-medium ${
            isActive ? 'text-blue-700' : 'text-gray-900 hover:text-blue-600'
          }`}>
            {conversation.title || "Untitled conversation"}
          </h3>
          <p className="text-sm text-gray-500 mt-1">
            {conversation.messageCount} message{conversation.messageCount !== 1 ? "s" : ""}
          </p>
          <p className="text-xs text-gray-400 mt-2">{formattedDate}</p>
        </Link>

        {onDelete && (
          <button
            onClick={(e) => {
              e.preventDefault();
              onDelete(conversation.id);
            }}
            className="text-gray-400 hover:text-red-500 p-1"
            title="Delete conversation"
          >
            <svg
              className="w-5 h-5"
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16"
              />
            </svg>
          </button>
        )}
      </div>
    </div>
  );
}
