import React, { useEffect, useState } from 'react';
import { useTextSelection } from '../context/TextSelectionContext';
import './TextSelectionHandler.css';

export default function TextSelectionHandler() {
  const { setSelectedText } = useTextSelection();
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });
  const [currentSelection, setCurrentSelection] = useState<string>('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      if (selectedText && selectedText.length > 0) {
        // Get selection coordinates
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setCurrentSelection(selectedText);
          setButtonPosition({
            top: rect.bottom + window.scrollY + 5,
            left: rect.left + window.scrollX + rect.width / 2
          });
          setShowButton(true);
        }
      } else {
        setShowButton(false);
      }
    };

    // Listen for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const handleAskAboutThis = () => {
    const MAX_LENGTH = 2000;
    let truncatedText = currentSelection;
    let wasTruncated = false;

    if (currentSelection.length > MAX_LENGTH) {
      truncatedText = currentSelection.substring(0, MAX_LENGTH);
      wasTruncated = true;
    }

    setSelectedText({
      text: truncatedText,
      source_url: window.location.href,
      chapter_id: extractChapterId(window.location.pathname)
    });

    setShowButton(false);

    if (wasTruncated) {
      alert(`Selected text was truncated to ${MAX_LENGTH} characters.`);
    }
  };

  const handleDismiss = () => {
    setShowButton(false);
    window.getSelection()?.removeAllRanges();
  };

  const extractChapterId = (pathname: string): string | undefined => {
    // Extract chapter ID from URL like /docs/module-1-ros2/01-ros2-basics
    const match = pathname.match(/\/docs\/(.+)/);
    return match ? match[1] : undefined;
  };

  if (!showButton) {
    return null;
  }

  return (
    <div
      className="text-selection-popup"
      style={{
        position: 'absolute',
        top: `${buttonPosition.top}px`,
        left: `${buttonPosition.left}px`,
        transform: 'translateX(-50%)'
      }}
    >
      <button
        className="ask-about-button"
        onClick={handleAskAboutThis}
        title="Ask AI about this text"
      >
        Ask about this
      </button>
      <button
        className="dismiss-button"
        onClick={handleDismiss}
        title="Dismiss"
      >
        &times;
      </button>
    </div>
  );
}
