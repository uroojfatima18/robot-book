import { ConversationList } from "@/components/chat/ConversationList";

export default function HistoryPage() {
  return (
    <div className="max-w-3xl mx-auto">
      <div className="flex justify-between items-center mb-6">
        <h1 className="text-2xl font-bold text-gray-900">Conversation History</h1>
      </div>
      <ConversationList />
    </div>
  );
}
