import time
import logging
from typing import List, Dict, Any, Optional
from openai import OpenAI
from openai.types.chat import ChatCompletion
from ..config import settings

logger = logging.getLogger(__name__)


class OpenAIProxy:
    """
    Proxy for OpenAI API with exponential backoff and rate limiting.
    Centralizes all OpenAI API calls for the application.
    """
    
    def __init__(self):
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.embedding_model = settings.openai_embedding_model
        self.chat_model = settings.openai_model
        
    def create_embedding(self, text: str) -> List[float]:
        """
        Create embedding for a single text using exponential backoff.
        
        Args:
            text: Input text to embed
            
        Returns:
            List of floats representing the embedding vector (1536 dimensions)
        """
        max_retries = 3
        retry_delay = 1  # seconds
        
        for attempt in range(max_retries):
            try:
                response = self.client.embeddings.create(
                    model=self.embedding_model,
                    input=text
                )
                return response.data[0].embedding
                
            except Exception as e:
                if attempt < max_retries - 1:
                    wait_time = retry_delay * (2 ** attempt)
                    logger.warning(f"OpenAI embedding error (attempt {attempt + 1}/{max_retries}): {e}. Retrying in {wait_time}s...")
                    time.sleep(wait_time)
                else:
                    logger.error(f"OpenAI embedding failed after {max_retries} attempts: {e}")
                    raise
    
    def create_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for multiple texts in a single API call.
        
        Args:
            texts: List of input texts
            
        Returns:
            List of embedding vectors
        """
        max_retries = 3
        retry_delay = 1
        
        for attempt in range(max_retries):
            try:
                response = self.client.embeddings.create(
                    model=self.embedding_model,
                    input=texts
                )
                return [item.embedding for item in response.data]
                
            except Exception as e:
                if attempt < max_retries - 1:
                    wait_time = retry_delay * (2 ** attempt)
                    logger.warning(f"OpenAI batch embedding error (attempt {attempt + 1}/{max_retries}): {e}. Retrying in {wait_time}s...")
                    time.sleep(wait_time)
                else:
                    logger.error(f"OpenAI batch embedding failed after {max_retries} attempts: {e}")
                    raise
    
    def chat_completion(
        self,
        messages: List[Dict[str, str]],
        temperature: float = 0.7,
        max_tokens: Optional[int] = None,
        functions: Optional[List[Dict[str, Any]]] = None
    ) -> ChatCompletion:
        """
        Create chat completion using OpenAI API.
        
        Args:
            messages: List of message dicts with 'role' and 'content'
            temperature: Sampling temperature (0-2)
            max_tokens: Maximum tokens in response
            functions: Optional function calling definitions
            
        Returns:
            ChatCompletion response object
        """
        max_retries = 3
        retry_delay = 1
        
        for attempt in range(max_retries):
            try:
                kwargs = {
                    "model": self.chat_model,
                    "messages": messages,
                    "temperature": temperature,
                }
                
                if max_tokens:
                    kwargs["max_tokens"] = max_tokens
                    
                if functions:
                    kwargs["functions"] = functions
                    kwargs["function_call"] = "auto"
                
                response = self.client.chat.completions.create(**kwargs)
                return response
                
            except Exception as e:
                if attempt < max_retries - 1:
                    wait_time = retry_delay * (2 ** attempt)
                    logger.warning(f"OpenAI chat completion error (attempt {attempt + 1}/{max_retries}): {e}. Retrying in {wait_time}s...")
                    time.sleep(wait_time)
                else:
                    logger.error(f"OpenAI chat completion failed after {max_retries} attempts: {e}")
                    raise
    
    def check_connection(self) -> bool:
        """
        Verify OpenAI API connection.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Simple test: create a small embedding
            self.create_embedding("test")
            return True
        except Exception as e:
            logger.error(f"OpenAI connection check failed: {e}")
            return False


# Global instance
openai_proxy = OpenAIProxy()
