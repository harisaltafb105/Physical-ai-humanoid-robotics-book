import React, { useState, useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import './ChatWithBook.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: any[];
  created_at: string;
}

interface ChatWithBookProps {
  fullScreen?: boolean;
  dedicatedPage?: boolean;
}

export default function ChatWithBook({ fullScreen = false, dedicatedPage = false }: ChatWithBookProps) {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isMinimized, setIsMinimized] = useState(!fullScreen);

  // Text selection feature disabled for now
  const selectedText = null;
  const clearSelection = () => {};

  const messagesEndRef = useRef<HTMLDivElement>(null);
  // Use window location to determine API URL, fallback to localhost for development
  const API_URL = typeof window !== 'undefined' && (window as any).API_URL
    ? (window as any).API_URL
    : 'http://localhost:8000';
  
  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);
  
  // Create conversation on mount
  useEffect(() => {
    if (!conversationId) {
      createConversation();
    }
  }, []);
  
  const createConversation = async () => {
    try {
      const response = await fetch(`${API_URL}/api/chat/conversations`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ language: 'en' })
      });
      
      if (!response.ok) {
        throw new Error('Failed to create conversation');
      }
      
      const data = await response.json();
      setConversationId(data.conversation_id);
    } catch (err) {
      setError('Failed to initialize chat. Please refresh the page.');
      console.error('Error creating conversation:', err);
    }
  };

  const sendMessage = async () => {
    if (!input.trim() || !conversationId || loading) {
      return;
    }

    setLoading(true);
    setError(null);

    const userQuery = input;
    setInput('');

    // Include selected text if available
    const requestBody: any = { content: userQuery };
    if (selectedText) {
      requestBody.selected_context = selectedText;
    }

    try {
      const response = await fetch(
        `${API_URL}/api/chat/conversations/${conversationId}/messages`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(requestBody)
        }
      );

      if (!response.ok) {
        if (response.status === 429) {
          throw new Error('Rate limit exceeded. Please wait a moment.');
        }
        throw new Error('Failed to send message');
      }

      const data = await response.json();

      setMessages(prev => [
        ...prev,
        data.user_message,
        data.assistant_message
      ]);

    } catch (err: any) {
      setError(err.message || 'Failed to send message. Please try again.');
      console.error('Error sending message:', err);
      setInput(userQuery);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  if (isMinimized && !fullScreen) {
    return (
      <button
        className="chat-widget-button"
        onClick={() => setIsMinimized(false)}
        title="Open AI Chatbot"
      >
        Chat with Book
      </button>
    );
  }

  return (
    <div className={`chat-widget ${fullScreen ? 'fullscreen' : 'floating'}`}>
      <div className="chat-header">
        <h3>AI Book Assistant</h3>
        {!fullScreen && (
          <button
            className="minimize-button"
            onClick={() => setIsMinimized(true)}
          >
            &times;
          </button>
        )}
      </div>

      {error && (
        <div className="chat-error">
          {error}
          <button onClick={() => setError(null)}>&times;</button>
        </div>
      )}

      <div className="chat-messages">
        {messages.length === 0 && (
          <div className="welcome-message">
            <p>Ask me anything about Physical AI, ROS2, Gazebo, Isaac Sim, or VLA!</p>
          </div>
        )}

        {messages.map(msg => (
          <ChatMessage
            key={msg.id}
            role={msg.role}
            content={msg.content}
            timestamp={msg.created_at}
            citations={msg.citations}
          />
        ))}

        {loading && (
          <div className="loading-message">
            <span className="loading-dots">Thinking...</span>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <div className="chat-input-container">
        {selectedText && (
          <div className="selected-text-display">
            <div className="selected-text-header">
              <span>Selected text:</span>
              <button
                className="clear-selection-button"
                onClick={clearSelection}
                title="Clear selection"
              >
                &times;
              </button>
            </div>
            <div className="selected-text-content">
              {selectedText.text.length > 200
                ? `${selectedText.text.substring(0, 200)}...`
                : selectedText.text}
            </div>
            <div className="selected-text-info">
              {selectedText.text.length} characters
              {selectedText.text.length > 2000 && ' (truncated to 2000)'}
            </div>
          </div>
        )}

        <div className="chat-input-row">
          <textarea
            className="chat-input"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder={
              selectedText
                ? "Ask a question about the selected text..."
                : "Ask a question about the book..."
            }
            disabled={loading || !conversationId}
            rows={2}
          />
          <button
            className="send-button"
            onClick={sendMessage}
            disabled={loading || !input.trim() || !conversationId}
          >
            {loading ? 'Sending...' : 'Send'}
          </button>
        </div>
      </div>
    </div>
  );
}
