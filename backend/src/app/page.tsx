import Link from "next/link";
import { Navigation } from "@/components/ui/Navigation";

export default function HomePage() {
  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100">
      <Navigation />
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-16">
        <div className="text-center mb-16">
          <h1 className="text-5xl md:text-6xl font-bold text-gray-900 mb-6">
            Robotics Book
            <span className="block text-blue-600">AI Assistant</span>
          </h1>
          <p className="text-xl text-gray-600 max-w-3xl mx-auto mb-8">
            Get instant answers to your questions about ROS 2, digital twins, AI-powered robots, and more.
            Powered by advanced RAG technology with verified book content.
          </p>

          <div className="flex flex-col sm:flex-row justify-center gap-4">
            <Link
              href="/signup"
              className="bg-blue-600 text-white px-8 py-4 rounded-lg font-medium hover:bg-blue-700 transition-colors text-lg shadow-lg hover:shadow-xl"
            >
              Get Started Free
            </Link>
            <Link
              href="/login"
              className="bg-white text-gray-800 px-8 py-4 rounded-lg font-medium border border-gray-200 hover:bg-gray-50 transition-colors text-lg shadow-sm"
            >
              Sign In
            </Link>
          </div>
        </div>

        <div className="grid md:grid-cols-3 gap-8 mb-16">
          <div className="bg-white rounded-xl shadow-lg p-8 hover:shadow-xl transition-shadow">
            <div className="text-4xl mb-4">🤖</div>
            <h2 className="text-xl font-semibold text-gray-900 mb-3">
              AI-Powered Answers
            </h2>
            <p className="text-gray-600 mb-4">
              Get instant, accurate answers from verified book content using advanced RAG technology.
            </p>
            <ul className="text-sm text-gray-500 space-y-1">
              <li>• Context-aware responses</li>
              <li>• Chapter references included</li>
              <li>• Real-time streaming answers</li>
            </ul>
          </div>

          <div className="bg-white rounded-xl shadow-lg p-8 hover:shadow-xl transition-shadow">
            <div className="text-4xl mb-4">💬</div>
            <h2 className="text-xl font-semibold text-gray-900 mb-3">
              Interactive Chat
            </h2>
            <p className="text-gray-600 mb-4">
              Engage in natural conversations about robotics concepts, code, and best practices.
            </p>
            <ul className="text-sm text-gray-500 space-y-1">
              <li>• Multi-turn conversations</li>
              <li>• Follow-up questions</li>
              <li>• Code explanations</li>
            </ul>
          </div>

          <div className="bg-white rounded-xl shadow-lg p-8 hover:shadow-xl transition-shadow">
            <div className="text-4xl mb-4">📚</div>
            <h2 className="text-xl font-semibold text-gray-900 mb-3">
              Book Knowledge
            </h2>
            <p className="text-gray-600 mb-4">
              Access comprehensive knowledge from the Physical AI & Humanoid Robotics textbook.
            </p>
            <ul className="text-sm text-gray-500 space-y-1">
              <li>• ROS 2 fundamentals</li>
              <li>• Navigation & SLAM</li>
              <li>• Computer vision</li>
            </ul>
          </div>
        </div>

        <div className="bg-white rounded-2xl shadow-xl p-8 md:p-12 mb-16">
          <div className="max-w-4xl mx-auto text-center">
            <h2 className="text-3xl font-bold text-gray-900 mb-4">
              Learn Robotics Faster
            </h2>
            <p className="text-lg text-gray-600 mb-8">
              Whether you're a beginner or experienced developer, our AI assistant helps you understand
              complex robotics concepts through interactive Q&A sessions.
            </p>
            <div className="grid md:grid-cols-2 gap-8 text-left">
              <div>
                <h3 className="font-semibold text-gray-900 mb-2">For Students</h3>
                <ul className="text-gray-600 space-y-1">
                  <li>• Clarify difficult concepts</li>
                  <li>• Get homework help</li>
                  <li>• Prepare for exams</li>
                </ul>
              </div>
              <div>
                <h3 className="font-semibold text-gray-900 mb-2">For Developers</h3>
                <ul className="text-gray-600 space-y-1">
                  <li>• Debug ROS 2 code</li>
                  <li>• Learn best practices</li>
                  <li>• Find implementation examples</li>
                </ul>
              </div>
            </div>
          </div>
        </div>

        <div className="text-center">
          <div className="inline-flex flex-col sm:flex-row gap-4">
            <Link
              href="/signup"
              className="bg-blue-600 text-white px-8 py-4 rounded-lg font-medium hover:bg-blue-700 transition-colors text-lg shadow-lg hover:shadow-xl"
            >
              Start Learning Today
            </Link>
            <Link
              href="/chat"
              className="bg-white text-gray-800 px-8 py-4 rounded-lg font-medium border border-gray-200 hover:bg-gray-50 transition-colors text-lg shadow-sm"
            >
              Try Demo Chat
            </Link>
          </div>
        </div>

        <div className="mt-16 text-center text-sm text-gray-500">
          <p>Powered by AI • Answers from verified book content only • Secure & Private</p>
        </div>
      </div>
    </div>
  );
}
