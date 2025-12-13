from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional


class DocumentChunk(BaseModel):
    """Pydantic model for Qdrant document chunks."""
    
    chunk_id: str = Field(..., description="Unique chunk identifier")
    chapter_id: str = Field(..., description="Reference to source chapter")
    module_name: str = Field(..., description="Module name (e.g., Module 1: ROS2)")
    chapter_title: str = Field(..., description="Chapter title")
    content: str = Field(..., description="Chunk text content")
    heading_hierarchy: List[str] = Field(default_factory=list, description="Heading path")
    has_code: bool = Field(default=False, description="Contains code blocks")
    code_language: Optional[str] = Field(None, description="Programming language if has_code")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional metadata")
    
    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "ros2-basics_0",
                "chapter_id": "module-1-ros2/01-ros2-basics-first-node",
                "module_name": "Module 1: ROS2",
                "chapter_title": "ROS2 Basics: Your First Node",
                "content": "ROS2 is the second generation...",
                "heading_hierarchy": ["Introduction", "What is ROS2"],
                "has_code": True,
                "code_language": "python",
                "metadata": {"line_start": 10, "line_end": 45}
            }
        }


class SearchResult(BaseModel):
    """Search result from vector database."""
    
    chunk_id: str
    chapter_id: str
    chapter_title: str
    section: str
    content: str
    score: float
    has_code: bool = False
    
    @classmethod
    def from_qdrant_point(cls, point):
        """Create SearchResult from Qdrant ScoredPoint."""
        payload = point.payload
        section = " > ".join(payload.get("heading_hierarchy", []))
        
        return cls(
            chunk_id=point.id,
            chapter_id=payload.get("chapter_id", ""),
            chapter_title=payload.get("chapter_title", ""),
            section=section,
            content=payload.get("content", ""),
            score=point.score,
            has_code=payload.get("has_code", False)
        )


class Citation(BaseModel):
    """Citation model for message citations."""
    
    chapter_id: str
    chapter_title: str
    section: str
    relevance_score: float
    excerpt: str
    
    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": "module-1-ros2/01-ros2-basics-first-node",
                "chapter_title": "ROS2 Basics: Your First Node",
                "section": "Creating a Publisher",
                "relevance_score": 0.92,
                "excerpt": "To create a publisher in ROS2..."
            }
        }
