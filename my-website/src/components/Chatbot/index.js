import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import { useAuth } from '../AuthContext';
import { useChatAPI } from '../../hooks/useChatAPI';
import styles from './styles.module.css';

export default function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const { user } = useAuth();
  const { messages, streamingContent, isLoading, error, sendMessage } = useChatAPI();

  const [inputText, setInputText] = useState("");
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen, streamingContent]);

  const handleSend = (e) => {
    e.preventDefault();
    if (!inputText.trim() || isLoading) return;

    sendMessage(inputText);
    setInputText("");
  };

  return (
    <div className={styles.chatbotWidget}>
      {isOpen && (
        <div className={styles.chatCard}>
          {/* Header */}
          <div className={styles.cardHeader}>
            <div className={styles.headerInfo}>
              <img src="/img/robot-avatar.png" alt="AI" className={styles.headerAvatar} />
              <div className={styles.headerTexts}>
                <span className={styles.headerName}>AI ASSISTANT</span>
                <span className={styles.headerStatus}>
                  {isLoading ? "Thinking..." : "Online"}
                </span>
              </div>
            </div>
            <button className={styles.closeBtn} onClick={() => setIsOpen(false)}>✕</button>
          </div>

          {/* Messages */}
          <div className={styles.msgBody}>
            {messages.length === 0 && !streamingContent && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book.</p>
                <p>Ask me anything about:</p>
                <ul>
                  <li>ROS 2 fundamentals & nervous system</li>
                  <li>Digital twins & simulation</li>
                  <li>AI-powered robot brains</li>
                  <li>Navigation, SLAM & perception</li>
                  <li>Adaptive robotics & learning</li>
                </ul>
              </div>
            )}

            {messages.map(msg => (
              <div key={msg.id} className={msg.role === 'bot' || msg.role === 'assistant' ? styles.msgBot : styles.msgUser}>
                {msg.content}
              </div>
            ))}

            {streamingContent && (
              <div className={styles.msgBot}>
                {streamingContent}
                <span className={styles.cursor}>▊</span>
              </div>
            )}

            {error && (
              <div className={styles.msgError}>
                ⚠️ {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputArea}>
            <form className={styles.inputWrapper} onSubmit={handleSend}>
              <input
                className={styles.inputField}
                placeholder="Ask about the book..."
                value={inputText}
                onChange={(e) => setInputText(e.target.value)}
                disabled={isLoading}
              />
              <button type="submit" className={styles.sendBtn} disabled={isLoading || !inputText.trim()}>
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              </button>
            </form>
          </div>
        </div>
      )}

      {/* Floating Toggle Button */}
      <button className={styles.toggleButton} onClick={() => setIsOpen(!isOpen)}>
        {/* Using the generated avatar as the icon for the button too, or could be a different icon.
            The image showed a robot face in a circle. */}
        <img src="/img/robot-avatar.png" alt="Chat" className={styles.buttonIcon} />
        <div className={styles.statusDot} />
      </button>
    </div>
  );
}
