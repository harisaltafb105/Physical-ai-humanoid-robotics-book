import React, { createContext, useContext, useState, ReactNode } from 'react';

interface SelectedText {
  text: string;
  source_url: string;
  chapter_id?: string;
}

interface TextSelectionContextType {
  selectedText: SelectedText | null;
  setSelectedText: (text: SelectedText | null) => void;
  clearSelection: () => void;
}

const TextSelectionContext = createContext<TextSelectionContextType | undefined>(undefined);

export function TextSelectionProvider({ children }: { children: ReactNode }) {
  const [selectedText, setSelectedText] = useState<SelectedText | null>(null);

  const clearSelection = () => {
    setSelectedText(null);
  };

  return (
    <TextSelectionContext.Provider value={{ selectedText, setSelectedText, clearSelection }}>
      {children}
    </TextSelectionContext.Provider>
  );
}

export function useTextSelection() {
  const context = useContext(TextSelectionContext);
  if (context === undefined) {
    // Return default values when provider is not available
    return {
      selectedText: null,
      setSelectedText: () => {},
      clearSelection: () => {}
    };
  }
  return context;
}
