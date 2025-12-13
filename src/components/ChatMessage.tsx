import React from 'react';

interface ChatMessageProps {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  citations?: Array<{
    chapter_id: string;
    chapter_title: string;
    section: string;
    relevance_score: number;
    excerpt: string;
  }>;
}

export default function ChatMessage({ role, content, timestamp, citations }: ChatMessageProps) {
  const isUser = role === 'user';
  
  return (
    <div className={`chat-message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className="message-header">
        <span className="message-role">{isUser ? 'You' : 'AI Assistant'}</span>
        <span className="message-timestamp">
          {new Date(timestamp).toLocaleTimeString()}
        </span>
      </div>
      
      <div className="message-content">
        {content}
      </div>
      
      {citations && citations.length > 0 && (
        <div className="message-citations">
          <strong>Sources:</strong>
          <ul>
            {citations.map((citation, index) => (
              <li key={index}>
                <a href={`/docs/${citation.chapter_id}`}>
                  {citation.chapter_title} - {citation.section}
                </a>
                <span className="citation-score">
                  ({Math.round(citation.relevance_score * 100)}% relevant)
                </span>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}
