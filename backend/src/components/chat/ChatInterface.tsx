'use client';

import { useState } from 'react';
import { ChatHistory } from './ChatHistory';
import { ChatInput } from './ChatInput';
import { useChat } from '@/hooks/useChat';
import { ConversationList } from './ConversationList';

interface ChatInterfaceProps {
  conversationId?: string;
}

export function ChatInterface({ conversationId: initialConversationId }: ChatInterfaceProps) {
  const [sidebarOpen, setSidebarOpen] = useState(false);

  const {
    messages,
    streamingContent,
    isLoading,
    error,
    sendMessage,
    conversationId,
  } = useChat(initialConversationId);

  return (
    <div className="flex h-full bg-white rounded-xl shadow-sm border border-gray-200 overflow-hidden">
      {/* Sidebar for conversation history */}
      <div
        className={`fixed inset-y-0 left-0 z-50 w-64 lg:w-80 bg-gray-50 border-r border-gray-200 transform transition-transform duration-300 ease-in-out md:translate-x-0 ${
          sidebarOpen ? 'translate-x-0' : '-translate-x-full'
        } md:static md:translate-x-0`}
      >
        <div className="flex flex-col h-full">
          <div className="p-4 border-b border-gray-200 flex items-center justify-between">
            <h2 className="text-lg font-semibold text-gray-800">Conversations</h2>
            <button
              onClick={() => setSidebarOpen(false)}
              className="md:hidden p-1 rounded hover:bg-gray-200 text-gray-600"
            >
              <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
          <div className="flex-1 overflow-y-auto p-3">
            <ConversationList currentConversationId={conversationId} />
          </div>
          <div className="p-4 border-t border-gray-200">
            <button className="w-full bg-primary-600 text-white py-2.5 px-4 rounded-lg hover:bg-primary-700 transition-colors text-sm font-medium shadow-sm">
              New Chat
            </button>
          </div>
        </div>
      </div>

      {/* Sidebar overlay for mobile */}
      {sidebarOpen && (
        <div
          className="fixed inset-0 z-40 bg-black bg-opacity-30 md:hidden"
          onClick={() => setSidebarOpen(false)}
        />
      )}

      {/* Main chat area */}
      <div className="flex-1 flex flex-col relative">
        {/* Chat header */}
        <div className="bg-primary-600 text-white px-4 py-3.5 flex items-center justify-between">
          <div className="flex items-center gap-2.5">
            <button
              onClick={() => setSidebarOpen(true)}
              className="p-1.5 rounded hover:bg-primary-500 md:hidden"
            >
              <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
              </svg>
            </button>
            <span className="text-xl hidden md:block">📚</span>
            <h2 className="font-semibold">Book Assistant</h2>
          </div>
          <div className="flex items-center gap-2">
            {conversationId && (
              <span className="text-xs text-primary-200 hidden sm:block">
                Conversation: {conversationId.substring(0, 8)}...
              </span>
            )}
            <button className="p-1.5 rounded hover:bg-primary-500">
              <svg className="w-4.5 h-4.5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 5v.01M12 12v.01M12 19v.01M12 6a1 1 0 110-2 1 1 0 010 2zm0 7a1 1 0 110-2 1 1 0 010 2zm0 7a1 1 0 110-2 1 1 0 010 2z" />
              </svg>
            </button>
          </div>
        </div>

        {/* Chat messages */}
        <ChatHistory messages={messages} streamingContent={streamingContent} />

        {/* Error display */}
        {error && (
          <div className="px-4 py-2.5 bg-red-50 text-red-700 text-sm border-t border-red-100">
            {error}
          </div>
        )}

        {/* Chat input */}
        <ChatInput onSend={sendMessage} disabled={isLoading} />
      </div>
    </div>
  );
}