import re
import logging
import hashlib
from pathlib import Path
from typing import List, Dict, Any, Tuple
from qdrant_client.models import PointStruct
from ..mcp.server import mcp_server

logger = logging.getLogger(__name__)


class DocumentChunk:
    """Represents a chunk of document content with metadata."""
    
    def __init__(
        self,
        chunk_id: str,
        chapter_id: str,
        module_name: str,
        chapter_title: str,
        content: str,
        heading_hierarchy: List[str],
        has_code: bool = False,
        code_language: str = None,
        metadata: Dict[str, Any] = None
    ):
        self.chunk_id = chunk_id
        self.chapter_id = chapter_id
        self.module_name = module_name
        self.chapter_title = chapter_title
        self.content = content
        self.heading_hierarchy = heading_hierarchy
        self.has_code = has_code
        self.code_language = code_language
        self.metadata = metadata or {}


class EmbeddingService:
    """Service for chunking markdown files and creating embeddings."""

    def __init__(self):
        self.chunk_size = 800  # tokens
        self.chunk_overlap = 200  # tokens
        self.max_chunks_per_file = 50

    def chunk_markdown(self, file_path: str) -> List[DocumentChunk]:
        """Parse and chunk a markdown file."""
        path = Path(file_path)

        if not path.exists():
            logger.error(f"File not found: {file_path}")
            return []

        with open(path, 'r', encoding='utf-8') as f:
            content = f.read()

        frontmatter, body = self._parse_frontmatter(content)
        chapter_id = frontmatter.get('id', path.stem)
        chapter_title = frontmatter.get('title', path.stem.replace('-', ' ').title())
        module_name = self._extract_module_name(file_path)

        sections = self._split_by_headings(body)

        chunks = []
        chunk_index = 0

        for section in sections:
            section_chunks = self._create_overlapping_chunks(
                section['content'],
                section['headings']
            )

            for chunk_content in section_chunks:
                has_code = '```' in chunk_content
                code_language = self._extract_code_language(chunk_content) if has_code else None

                chunk = DocumentChunk(
                    chunk_id=f"{chapter_id}_{chunk_index}",
                    chapter_id=chapter_id,
                    module_name=module_name,
                    chapter_title=chapter_title,
                    content=chunk_content,
                    heading_hierarchy=section['headings'],
                    has_code=has_code,
                    code_language=code_language,
                    metadata={"file_path": str(path), "chunk_index": chunk_index}
                )
                chunks.append(chunk)
                chunk_index += 1

        logger.info(f"Created {len(chunks)} chunks from {file_path}")
        return chunks

    def create_embeddings(self, chunks: List[DocumentChunk]) -> List[List[float]]:
        """Create embeddings for chunks using MCP OpenAI proxy."""
        texts = [chunk.content for chunk in chunks]

        try:
            embeddings = mcp_server.openai.create_embeddings_batch(texts)
            logger.info(f"Created {len(embeddings)} embeddings")
            return embeddings
        except Exception as e:
            logger.error(f"Failed to create embeddings: {e}")
            raise

    def upload_to_qdrant(self, chunks: List[DocumentChunk], embeddings: List[List[float]]) -> bool:
        """Upload chunks and embeddings to Qdrant using MCP proxy."""
        if len(chunks) != len(embeddings):
            logger.error(f"Chunk count ({len(chunks)}) doesn't match embedding count ({len(embeddings)})")
            return False

        points = []
        for chunk, embedding in zip(chunks, embeddings):
            point = PointStruct(
                id=chunk.chunk_id,
                vector=embedding,
                payload={
                    "chapter_id": chunk.chapter_id,
                    "module_name": chunk.module_name,
                    "chapter_title": chunk.chapter_title,
                    "content": chunk.content,
                    "heading_hierarchy": chunk.heading_hierarchy,
                    "has_code": chunk.has_code,
                    "code_language": chunk.code_language,
                    "metadata": chunk.metadata
                }
            )
            points.append(point)

        try:
            success = mcp_server.qdrant.upsert_vectors(points)
            return success
        except Exception as e:
            logger.error(f"Failed to upload to Qdrant: {e}")
            return False

    def _parse_frontmatter(self, content: str) -> Tuple[Dict[str, str], str]:
        """Extract frontmatter and body from markdown."""
        frontmatter = {}
        body = content

        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                fm_text = parts[1].strip()
                body = parts[2].strip()

                for line in fm_text.split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        frontmatter[key.strip()] = value.strip()

        return frontmatter, body

    def _split_by_headings(self, content: str) -> List[Dict[str, Any]]:
        """Split content by markdown headings."""
        sections = []
        current_headings = []
        current_content = []

        lines = content.split('\n')

        for line in lines:
            heading_match = re.match(r'^(#{1,6})\s+(.+)$', line)

            if heading_match:
                if current_content:
                    sections.append({
                        'headings': current_headings.copy(),
                        'content': '\n'.join(current_content)
                    })
                    current_content = []

                level = len(heading_match.group(1))
                heading_text = heading_match.group(2)
                current_headings = current_headings[:level-1] + [heading_text]

            current_content.append(line)

        if current_content:
            sections.append({
                'headings': current_headings.copy(),
                'content': '\n'.join(current_content)
            })

        return sections

    def _create_overlapping_chunks(self, content: str, headings: List[str]) -> List[str]:
        """Create chunks with overlap."""
        char_size = self.chunk_size * 4
        char_overlap = self.chunk_overlap * 4

        chunks = []
        start = 0

        while start < len(content):
            end = start + char_size
            chunk = content[start:end]

            if chunk.strip():
                chunks.append(chunk)

            start = end - char_overlap

            if start >= len(content):
                break

        return chunks if chunks else [content]

    def _extract_module_name(self, file_path: str) -> str:
        """Extract module name from file path."""
        path = Path(file_path)

        for part in path.parts:
            if part.startswith('module-'):
                return part.replace('-', ' ').title()

        return "General"

    def _extract_code_language(self, content: str) -> str:
        """Extract programming language from code block."""
        match = re.search(r'```(\w+)', content)
        return match.group(1) if match else None

    def calculate_file_hash(self, file_path: str) -> str:
        """Calculate MD5 hash of file for change detection."""
        with open(file_path, 'rb') as f:
            return hashlib.md5(f.read()).hexdigest()


embedding_service = EmbeddingService()
