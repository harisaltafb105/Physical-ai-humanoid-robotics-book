import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
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
  const { siteConfig } = useDocusaurusContext();
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

  // API URL RESOLUTION - strict environment-based logic
  // - localhost ONLY when hostname === "localhost"
  // - Production MUST have DOCUSAURUS_API_URL env var, NO fallback
  // - Missing production env var returns null to show clear error
  const getApiUrl = (): string | null => {
    // 1. Get from Docusaurus config (built-in DOCUSAURUS_API_URL env var)
    const configUrl = siteConfig.customFields?.apiUrl as string | undefined;

    // 2. Detect environment
    const hostname = typeof window !== 'undefined' ? window.location.hostname : '';
    const isLocalhost = hostname === 'localhost' || hostname === '127.0.0.1';

    // 3. Determine final URL
    let finalUrl: string | null;

    if (isLocalhost) {
      // Local development - use localhost ONLY when hostname is localhost
      finalUrl = 'http://localhost:8000';
      console.log('[ChatBot] Local development detected - using localhost:8000');
    } else if (configUrl) {
      // Production with proper config - use environment variable
      finalUrl = configUrl;
      console.log('[ChatBot] Production environment - using DOCUSAURUS_API_URL:', configUrl);
    } else {
      // CRITICAL: Production without config - cannot initialize
      console.error('[ChatBot] ✗✗✗ CRITICAL: Production environment detected but DOCUSAURUS_API_URL is not set!');
      console.error('[ChatBot] Please set DOCUSAURUS_API_URL environment variable in your deployment dashboard');
      console.error('[ChatBot] Example: DOCUSAURUS_API_URL=https://your-backend.railway.app');
      console.error('[ChatBot] Hostname:', hostname);
      finalUrl = null;
    }

    // 4. Debug logging
    console.log('[ChatBot] === API URL Resolution ===');
    console.log('[ChatBot] Hostname:', hostname || 'SSR');
    console.log('[ChatBot] Is Localhost:', isLocalhost);
    console.log('[ChatBot] Config URL:', configUrl || 'NOT SET');
    console.log('[ChatBot] FINAL API_URL:', finalUrl || 'NULL - MISSING CONFIG');
    console.log('[ChatBot] ========================');

    return finalUrl;
  };

  const API_URL = getApiUrl();

  // If API_URL is null (production without env var), show configuration error
  if (!API_URL) {
    return (
      <div className={`chat-widget ${fullScreen ? 'fullscreen' : 'floating'}`}>
        <div className="chat-header">
          <div className="chat-header-left">
            <div className="bot-avatar">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M12 2C10.8954 2 10 2.89543 10 4C10 5.10457 10.8954 6 12 6C13.1046 6 14 5.10457 14 4C14 2.89543 13.1046 2 12 2Z" fill="currentColor"/>
                <path d="M7 8C5.34315 8 4 9.34315 4 11V17C4 18.6569 5.34315 20 7 20H17C18.6569 20 20 18.6569 20 17V11C20 9.34315 18.6569 8 17 8H7ZM9 13C9 12.4477 9.44772 12 10 12C10.5523 12 11 12.4477 11 13C11 13.5523 10.5523 14 10 14C9.44772 14 9 13.5523 9 13ZM14 12C13.4477 12 13 12.4477 13 13C13 13.5523 13.4477 14 14 14C14.5523 14 15 13.5523 15 13C15 12.4477 14.5523 12 14 12Z" fill="currentColor"/>
              </svg>
            </div>
            <div className="chat-header-title">
              <h3>AI Book Assistant</h3>
              <span className="chat-status" style={{ color: '#ef4444' }}>Configuration Error</span>
            </div>
          </div>
        </div>
        <div className="chat-messages">
          <div className="chat-error" style={{ margin: '20px', position: 'relative' }}>
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M10 18C14.4183 18 18 14.4183 18 10C18 5.58172 14.4183 2 10 2C5.58172 2 2 5.58172 2 10C2 14.4183 5.58172 18 10 18Z" fill="#ef4444"/>
              <path d="M10 6V10M10 14H10.01" stroke="white" strokeWidth="2" strokeLinecap="round"/>
            </svg>
            <div>
              <strong>Configuration Required</strong>
              <p style={{ marginTop: '8px', fontSize: '14px' }}>
                The chat service is not configured. Please set the <code>DOCUSAURUS_API_URL</code> environment variable in your deployment settings.
              </p>
              <p style={{ marginTop: '8px', fontSize: '12px', opacity: 0.8 }}>
                Check the browser console for more details.
              </p>
            </div>
          </div>
        </div>
      </div>
    );
  }

  // Debug logging and test backend connection
  useEffect(() => {
    console.log('[ChatBot] Component mounted - Testing backend connectivity...');
    console.log('[ChatBot] Using API_URL:', API_URL);

    // Test backend connectivity
    fetch(`${API_URL}/`)
      .then(res => res.json())
      .then(data => {
        console.log('[ChatBot] ✓ Backend connected successfully!');
        console.log('[ChatBot] Backend root response:', data);
        console.log('[ChatBot] Backend CORS config:', data.cors_origins_list);
      })
      .catch(err => {
        console.error('[ChatBot] ✗ Backend connectivity test FAILED:', err);
        console.error('[ChatBot] Make sure backend is running at:', API_URL);
      });
  }, [API_URL]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Load or create conversation on mount
  useEffect(() => {
    const initializeChat = async () => {
      console.log('[ChatBot] === Initializing Chat ===');
      console.log('[ChatBot] Current API_URL:', API_URL);

      // Check localStorage for existing conversation
      const savedConversationId = localStorage.getItem('chat_conversation_id');

      if (savedConversationId) {
        console.log('[ChatBot] Found saved conversation ID:', savedConversationId);
        console.log('[ChatBot] Attempting to load previous messages from:', `${API_URL}/api/chat/conversations/${savedConversationId}/messages`);

        // Try to load previous messages
        try {
          const response = await fetch(
            `${API_URL}/api/chat/conversations/${savedConversationId}/messages`
          );

          console.log('[ChatBot] Load messages response status:', response.status);

          if (response.ok) {
            const previousMessages = await response.json();
            console.log('[ChatBot] ✓ Successfully loaded', previousMessages.length, 'previous messages');
            setMessages(previousMessages);
            setConversationId(savedConversationId);
            return; // Success - exit early
          } else {
            console.warn('[ChatBot] ✗ Could not load previous messages (status:', response.status, '), creating new conversation');
            localStorage.removeItem('chat_conversation_id');
          }
        } catch (err) {
          console.error('[ChatBot] ✗ Error loading messages:', err);
          console.log('[ChatBot] Clearing saved conversation and creating new one');
          localStorage.removeItem('chat_conversation_id');
        }
      } else {
        console.log('[ChatBot] No saved conversation found in localStorage');
      }

      // No saved conversation or loading failed - create new one
      console.log('[ChatBot] Creating new conversation...');
      await createConversation();
    };

    initializeChat();
  }, [API_URL]); // Re-initialize if API_URL changes

  const createConversation = async () => {
    try {
      const createUrl = `${API_URL}/api/chat/conversations`;
      console.log('[ChatBot] === Creating New Conversation ===');
      console.log('[ChatBot] POST to:', createUrl);
      console.log('[ChatBot] Request body:', { language: 'en' });

      const response = await fetch(createUrl, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ language: 'en' })
      });

      console.log('[ChatBot] Response status:', response.status);
      console.log('[ChatBot] Response ok:', response.ok);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('[ChatBot] ✗ Error response body:', errorText);
        throw new Error(`Failed to create conversation: ${response.status} ${errorText}`);
      }

      const data = await response.json();
      console.log('[ChatBot] ✓ Conversation created successfully!');
      console.log('[ChatBot] Conversation ID:', data.conversation_id);
      setConversationId(data.conversation_id);

      // Save to localStorage for persistence across sessions
      localStorage.setItem('chat_conversation_id', data.conversation_id);
      console.log('[ChatBot] Conversation ID saved to localStorage');
      console.log('[ChatBot] ============================');
    } catch (err) {
      console.error('[ChatBot] ✗✗✗ CRITICAL ERROR creating conversation:', err);
      console.error('[ChatBot] API_URL was:', API_URL);
      console.error('[ChatBot] Please check:');
      console.error('[ChatBot]   1. Backend is running');
      console.error('[ChatBot]   2. CORS is configured correctly');
      console.error('[ChatBot]   3. API_URL is correct for your environment');
      setError('Failed to initialize chat. Please refresh the page.');
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
        <svg className="chat-icon" width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M10 2C5.58172 2 2 5.58172 2 10C2 11.8919 2.64262 13.6314 3.72906 15.0079L2.37467 17.5042C2.21038 17.8109 2.37272 18.1909 2.70783 18.2888C2.79681 18.3144 2.89017 18.3199 2.98143 18.3049L6.85464 17.713C7.83011 18.2162 8.91885 18.5058 10.0725 18.5058C14.4908 18.5058 18.0725 14.924 18.0725 10.5058C18.0725 6.08747 14.4908 2.50575 10.0725 2.50575L10 2Z" fill="currentColor"/>
        </svg>
        <span>Chat with Book</span>
      </button>
    );
  }

  const startNewChat = async () => {
    localStorage.removeItem('chat_conversation_id');
    setMessages([]);
    setConversationId(null);
    setError(null);
    await createConversation();
  };

  return (
    <div className={`chat-widget ${fullScreen ? 'fullscreen' : 'floating'}`}>
      <div className="chat-header">
        <div className="chat-header-left">
          <div className="bot-avatar">
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 2C10.8954 2 10 2.89543 10 4C10 5.10457 10.8954 6 12 6C13.1046 6 14 5.10457 14 4C14 2.89543 13.1046 2 12 2Z" fill="currentColor"/>
              <path d="M7 8C5.34315 8 4 9.34315 4 11V17C4 18.6569 5.34315 20 7 20H17C18.6569 20 20 18.6569 20 17V11C20 9.34315 18.6569 8 17 8H7ZM9 13C9 12.4477 9.44772 12 10 12C10.5523 12 11 12.4477 11 13C11 13.5523 10.5523 14 10 14C9.44772 14 9 13.5523 9 13ZM14 12C13.4477 12 13 12.4477 13 13C13 13.5523 13.4477 14 14 14C14.5523 14 15 13.5523 15 13C15 12.4477 14.5523 12 14 12Z" fill="currentColor"/>
            </svg>
          </div>
          <div className="chat-header-title">
            <h3>AI Book Assistant</h3>
            <span className="chat-status">Online</span>
          </div>
        </div>
        <div className="chat-header-actions">
          <button
            className="new-chat-button"
            onClick={startNewChat}
            title="Start new conversation"
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M8 3V13M3 8H13" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
            </svg>
            <span>New Chat</span>
          </button>
          {!fullScreen && (
            <button
              className="minimize-button"
              onClick={() => setIsMinimized(true)}
              title="Minimize chat"
            >
              <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M4 10H16" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
              </svg>
            </button>
          )}
        </div>
      </div>

      {error && (
        <div className="chat-error">
          <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M10 18C14.4183 18 18 14.4183 18 10C18 5.58172 14.4183 2 10 2C5.58172 2 2 5.58172 2 10C2 14.4183 5.58172 18 10 18Z" fill="#ef4444"/>
            <path d="M10 6V10M10 14H10.01" stroke="white" strokeWidth="2" strokeLinecap="round"/>
          </svg>
          <span>{error}</span>
          <button onClick={() => setError(null)}>
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M15 5L5 15M5 5L15 15" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
            </svg>
          </button>
        </div>
      )}

      <div className="chat-messages">
        {messages.length === 0 && !loading && (
          <div className="welcome-message">
            <div className="welcome-icon">
              <svg width="64" height="64" viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
                <circle cx="32" cy="32" r="30" fill="#f0f4ff"/>
                <path d="M32 16C29.7909 16 28 17.7909 28 20C28 22.2091 29.7909 24 32 24C34.2091 24 36 22.2091 36 20C36 17.7909 34.2091 16 32 16Z" fill="#667eea"/>
                <path d="M22 28C18.6863 28 16 30.6863 16 34V44C16 47.3137 18.6863 50 22 50H42C45.3137 50 48 47.3137 48 44V34C48 30.6863 45.3137 28 42 28H22ZM26 38C26 36.8954 26.8954 36 28 36C29.1046 36 30 36.8954 30 38C30 39.1046 29.1046 40 28 40C26.8954 40 26 39.1046 26 38ZM36 36C34.8954 36 34 36.8954 34 38C34 39.1046 34.8954 40 36 40C37.1046 40 38 39.1046 38 38C38 36.8954 37.1046 36 36 36Z" fill="#667eea"/>
              </svg>
            </div>
            <h4>Welcome to AI Book Assistant!</h4>
            <p>Ask me anything about:</p>
            <div className="welcome-topics">
              <span className="topic-tag">Physical AI</span>
              <span className="topic-tag">ROS 2</span>
              <span className="topic-tag">Gazebo</span>
              <span className="topic-tag">Isaac Sim</span>
              <span className="topic-tag">VLA</span>
            </div>
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
            <div className="loading-avatar">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M12 2C10.8954 2 10 2.89543 10 4C10 5.10457 10.8954 6 12 6C13.1046 6 14 5.10457 14 4C14 2.89543 13.1046 2 12 2Z" fill="currentColor"/>
                <path d="M7 8C5.34315 8 4 9.34315 4 11V17C4 18.6569 5.34315 20 7 20H17C18.6569 20 20 18.6569 20 17V11C20 9.34315 18.6569 8 17 8H7ZM9 13C9 12.4477 9.44772 12 10 12C10.5523 12 11 12.4477 11 13C11 13.5523 10.5523 14 10 14C9.44772 14 9 13.5523 9 13ZM14 12C13.4477 12 13 12.4477 13 13C13 13.5523 13.4477 14 14 14C14.5523 14 15 13.5523 15 13C15 12.4477 14.5523 12 14 12Z" fill="currentColor"/>
              </svg>
            </div>
            <div className="loading-dots-container">
              <span className="loading-dot"></span>
              <span className="loading-dot"></span>
              <span className="loading-dot"></span>
            </div>
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
                : "Type your message..."
            }
            disabled={loading || !conversationId}
            rows={1}
          />
          <button
            className="send-button"
            onClick={sendMessage}
            disabled={loading || !input.trim() || !conversationId}
            title="Send message"
          >
            {loading ? (
              <svg className="loading-spinner" width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
                <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2" opacity="0.25"/>
                <path d="M10 2C14.4183 2 18 5.58172 18 10" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
              </svg>
            ) : (
              <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M2 10L18 2L10 18L8 11L2 10Z" fill="currentColor"/>
              </svg>
            )}
          </button>
        </div>
      </div>
    </div>
  );
}
