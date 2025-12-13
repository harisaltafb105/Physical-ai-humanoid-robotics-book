import logging
from typing import List, Dict, Any, Optional
from ..mcp.server import mcp_server
from ..models.chunk import SearchResult, Citation

logger = logging.getLogger(__name__)


class RAGService:
    """
    Retrieval-Augmented Generation service for chatbot.
    Handles vector search, response generation, and citation extraction.
    """
    
    def __init__(self):
        self.default_search_limit = 5
        self.max_context_chunks = 3
    
    def search_book_content(
        self,
        query: str,
        limit: int = None,
        filter_module: Optional[str] = None
    ) -> List[SearchResult]:
        """
        Search book content using vector similarity.
        
        Args:
            query: User query text
            limit: Maximum number of results (default: 5)
            filter_module: Optional module name filter
            
        Returns:
            List of SearchResult objects
        """
        if limit is None:
            limit = self.default_search_limit
        
        try:
            # Create embedding for query
            query_embedding = mcp_server.openai.create_embedding(query)
            
            # Search Qdrant
            results = mcp_server.qdrant.search_vectors(
                query_vector=query_embedding,
                limit=limit
            )
            
            # Convert to SearchResult objects
            search_results = [
                SearchResult.from_qdrant_point(point)
                for point in results
            ]
            
            logger.info(f"Found {len(search_results)} results for query: {query[:50]}...")
            return search_results
            
        except Exception as e:
            logger.error(f"Error searching book content: {e}")
            return []
    
    def generate_response(
        self,
        query: str,
        retrieved_chunks: List[SearchResult],
        conversation_history: List[Dict[str, str]] = None,
        selected_context: Optional[str] = None
    ) -> str:
        """
        Generate response using OpenAI chat completion.
        
        Args:
            query: User query
            retrieved_chunks: List of retrieved document chunks
            conversation_history: Previous messages (optional)
            selected_context: User-selected text context (optional)
            
        Returns:
            Generated response text
        """
        # Build context from retrieved chunks
        context_parts = []
        for i, chunk in enumerate(retrieved_chunks[:self.max_context_chunks], 1):
            context_parts.append(
                f"[Source {i}: {chunk.chapter_title} - {chunk.section}]\n{chunk.content}\n"
            )
        
        context = "\n".join(context_parts)
        
        # Build system message
        system_message = f"""You are an AI assistant for the Physical AI & Humanoid Robotics book. 
Answer questions based on the provided book content. Always cite your sources.

Book Content:
{context}
"""
        
        if selected_context:
            system_message += f"

User Selected Text:
{selected_context}
"
        
        # Build messages
        messages = [{"role": "system", "content": system_message}]
        
        # Add conversation history
        if conversation_history:
            messages.extend(conversation_history[-5:])  # Last 5 messages for context
        
        # Add current query
        messages.append({"role": "user", "content": query})
        
        try:
            # Generate response
            response = mcp_server.openai.chat_completion(
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )
            
            answer = response.choices[0].message.content
            logger.info(f"Generated response ({len(answer)} chars)")
            return answer
            
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            return "I apologize, but I encountered an error generating a response. Please try again."
    
    def extract_citations(self, chunks: List[SearchResult]) -> List[Dict[str, Any]]:
        """
        Extract citations from search results for JSONB storage.
        
        Args:
            chunks: List of SearchResult objects
            
        Returns:
            List of citation dictionaries
        """
        citations = []
        
        for chunk in chunks:
            citation = {
                "chapter_id": chunk.chapter_id,
                "chapter_title": chunk.chapter_title,
                "section": chunk.section,
                "relevance_score": round(chunk.score, 3),
                "excerpt": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
            }
            citations.append(citation)
        
        return citations
    
    def ask_question(
        self,
        query: str,
        conversation_history: List[Dict[str, str]] = None,
        selected_context: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Complete RAG pipeline: search, generate, cite.
        
        Args:
            query: User question
            conversation_history: Previous messages
            selected_context: User-selected text
            
        Returns:
            Dict with response, citations, and metadata
        """
        # Step 1: Search
        chunks = self.search_book_content(query)
        
        if not chunks:
            return {
                "response": "I couldn't find relevant information in the book to answer your question. Could you rephrase or ask about a different topic?",
                "citations": [],
                "chunks_found": 0
            }
        
        # Step 2: Generate
        response = self.generate_response(
            query=query,
            retrieved_chunks=chunks,
            conversation_history=conversation_history,
            selected_context=selected_context
        )
        
        # Step 3: Extract citations
        citations = self.extract_citations(chunks)
        
        return {
            "response": response,
            "citations": citations,
            "chunks_found": len(chunks)
        }


# Global instance
rag_service = RAGService()
