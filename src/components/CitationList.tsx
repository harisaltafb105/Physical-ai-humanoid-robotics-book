import React from 'react';

interface Citation {
  chapter_id: string;
  chapter_title: string;
  section: string;
  relevance_score: number;
  excerpt: string;
}

interface CitationListProps {
  citations: Citation[];
}

export default function CitationList({ citations }: CitationListProps) {
  if (!citations || citations.length === 0) {
    return null;
  }
  
  return (
    <div className="citation-list">
      <h4>Sources from the book:</h4>
      <div className="citations">
        {citations.map((citation, index) => (
          <div key={index} className="citation-item">
            <div className="citation-header">
              <a 
                href={`/docs/${citation.chapter_id}`}
                className="citation-link"
                target="_blank"
                rel="noopener noreferrer"
              >
                <strong>{citation.chapter_title}</strong>
              </a>
              <span className="citation-score">
                {Math.round(citation.relevance_score * 100)}% relevant
              </span>
            </div>
            
            {citation.section && (
              <div className="citation-section">
                Section: {citation.section}
              </div>
            )}
            
            <div className="citation-excerpt">
              {citation.excerpt}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
