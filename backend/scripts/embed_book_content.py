import os
import sys
import glob
import json
import logging
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.services.embedding_service import embedding_service
from dotenv import load_dotenv

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

load_dotenv()


def process_chapter(file_path: str, processed_hashes: dict) -> bool:
    """Process a single chapter file."""
    try:
        file_hash = embedding_service.calculate_file_hash(file_path)
        
        if file_path in processed_hashes and processed_hashes[file_path] == file_hash:
            logger.info(f"[SKIP] {file_path} (unchanged)")
            return False
        
        logger.info(f"[PROCESS] {file_path}")
        
        chunks = embedding_service.chunk_markdown(file_path)
        if not chunks:
            logger.warning(f"No chunks created from {file_path}")
            return False
        
        embeddings = embedding_service.create_embeddings(chunks)
        success = embedding_service.upload_to_qdrant(chunks, embeddings)
        
        if success:
            processed_hashes[file_path] = file_hash
            logger.info(f"[SUCCESS] Uploaded {len(chunks)} chunks from {file_path}")
            return True
        else:
            logger.error(f"[FAIL] Failed to upload chunks from {file_path}")
            return False
            
    except Exception as e:
        logger.error(f"[ERROR] Processing {file_path}: {e}")
        return False


def main():
    """Main embedding pipeline."""
    docs_dir = Path("..") / "docs"
    cache_file = Path(".embedding_cache.json")
    
    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        sys.exit(1)
    
    processed_hashes = {}
    if cache_file.exists():
        with open(cache_file, "r") as f:
            processed_hashes = json.load(f)
    
    md_files = glob.glob(str(docs_dir / "**/*.md"), recursive=True)
    logger.info(f"Found {len(md_files)} markdown files")
    
    total_processed = 0
    total_skipped = 0
    total_failed = 0
    
    for md_file in md_files:
        if process_chapter(md_file, processed_hashes):
            total_processed += 1
        elif md_file in processed_hashes:
            total_skipped += 1
        else:
            total_failed += 1
    
    with open(cache_file, "w") as f:
        json.dump(processed_hashes, f, indent=2)
    
    logger.info("=" * 60)
    logger.info(f"Embedding pipeline complete!")
    logger.info(f"   Processed: {total_processed}")
    logger.info(f"   Skipped (unchanged): {total_skipped}")
    logger.info(f"   Failed: {total_failed}")
    logger.info("=" * 60)


if __name__ == "__main__":
    main()
