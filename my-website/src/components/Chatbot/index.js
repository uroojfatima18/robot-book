import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import { useAuth } from '../AuthContext';
import { useChatAPI } from '../../hooks/useChatAPI';
import styles from './styles.module.css';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Link from '@docusaurus/Link';

export default function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [isExpanded, setIsExpanded] = useState(false);
  const auth = useAuth();
  const user = auth ? auth.user : null;
  
  const chatAPI = useChatAPI();
  const { messages, streamingContent, isLoading, error, sendMessage } = chatAPI || {
    messages: [], streamingContent: "", isLoading: false, error: null, sendMessage: () => {}
  };

  const [inputText, setInputText] = useState("");
  const [selectedText, setSelectedText] = useState("");
  const messagesEndRef = useRef(null);
  
  const botAvatarUrl = useBaseUrl('img/robot-avatar.png');

  // Handle text selection from the book
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection().toString().trim();
      if (selection) {
        setSelectedText(selection);
      }
    };

    document.addEventListener("selectionchange", handleSelection);
    return () => document.removeEventListener("selectionchange", handleSelection);
  }, []);

  const handleSend = (e) => {
    e.preventDefault();
    if (!inputText.trim() || isLoading) return;

    sendMessage(inputText, selectedText);
    setInputText("");
    setSelectedText(""); // Clear selection after sending
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen, streamingContent]);

  return (
    <div className={styles.chatbotWidget}>
      {isOpen && (
        <div className={clsx(styles.chatCard, isExpanded && styles.expanded)}>
          {/* Header */}
          <div className={styles.cardHeader}>
            <div className={styles.headerInfo}>
              <img src={botAvatarUrl} alt="AI" className={styles.headerAvatar} />
              <div className={styles.headerTexts}>
                <span className={styles.headerName}>AI ASSISTANT</span>
                <span className={styles.headerStatus}>
                  {isLoading ? "Thinking..." : "Online"}
                </span>
              </div>
            </div>
            <div className={styles.headerActions}>
              <button 
                className={styles.actionBtn} 
                onClick={() => setIsExpanded(!isExpanded)}
                title={isExpanded ? "Shrink" : "Expand"}
              >
                {isExpanded ? '⬂' : '⬃'}
              </button>
              <button className={styles.closeBtn} onClick={() => setIsOpen(false)}>✕</button>
            </div>
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
                <p className={styles.tip}>💡 <b>Tip:</b> Select any text in the book to ask a specific question about it!</p>
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
            {selectedText && (
              <div className={styles.selectionIndicator}>
                <div className={styles.selectionHeader}>
                  <span>Context: "<i>{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}</i>"</span>
                  <button onClick={() => setSelectedText('')}>✕</button>
                </div>
              </div>
            )}
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
        <img src={botAvatarUrl} alt="Chat" className={styles.buttonIcon} />
        <div className={styles.statusDot} />
      </button>
    </div>
  );
}
