import React from 'react';
import { TextSelectionProvider } from '../context/TextSelectionContext';
import TextSelectionHandler from '../components/TextSelectionHandler';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <TextSelectionProvider>
      {children}
      <TextSelectionHandler />
    </TextSelectionProvider>
  );
}
