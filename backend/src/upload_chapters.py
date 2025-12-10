"""
Upload script to chunk chapters and store in Qdrant
This script processes all chapters in the textbook and stores them in the vector database for RAG functionality
"""
import os
import sys
import json
import asyncio
from typing import List, Dict, Any
from pathlib import Path

# Add the backend src directory to the path to import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

from rag.vector_db import vector_db, chunker
from rag.cohere_embedder import embedding_service
from models.chapter import Chapter
from database import SessionLocal


def process_chapter_content(chapter_id: int, title: str, content: str) -> Dict[str, Any]:
    """
    Process a single chapter: chunk it, generate embeddings, and store in Qdrant
    """
    print(f"Processing chapter: {title} (ID: {chapter_id})")
    
    # Chunk the content
    chunks = chunker.chunk_text(content)
    print(f"  Created {len(chunks)} chunks")
    
    # Process each chunk
    for chunk in chunks:
        try:
            # Generate embedding for the chunk
            embedding = embedding_service.get_embedding_for_content(
                chunk['text'],
                language="en",
                content_type="document"
            )
            
            # Prepare metadata
            metadata = {
                'chapter_id': chapter_id,
                'chapter_title': title,
                'chunk_position': chunk['metadata']['position'],
                'word_count': chunk['metadata']['word_count'],
                'text': chunk['text']  # Store the text in metadata for retrieval
            }
            
            # Generate a unique content ID for this chunk
            chunk_id = f"{chapter_id}_chunk_{chunk['metadata']['position']}"
            
            # Store in vector database
            vector_db.store_embeddings(
                content_id=chunk_id,
                embedding=embedding,
                metadata=metadata
            )
            
            print(f"    Stored chunk {chunk['metadata']['position']} in vector DB")
            
        except Exception as e:
            print(f"    Error processing chunk {chunk.get('metadata', {}).get('position', 'unknown')}: {str(e)}")
    
    return {
        "chapter_id": chapter_id,
        "chapter_title": title,
        "chunks_processed": len(chunks),
        "status": "success"
    }


def get_all_chapters() -> List[Dict[str, Any]]:
    """
    Retrieve all chapters from the database
    """
    db = SessionLocal()
    try:
        chapters = db.query(Chapter).filter(Chapter.is_active == True).all()
        return [{
            'id': chapter.id,
            'title': chapter.title,
            'content': chapter.content,
            'module_id': chapter.module_id
        } for chapter in chapters]
    finally:
        db.close()


def main():
    """
    Main function to process all chapters and store in Qdrant
    """
    print("Starting chapter upload and vectorization process...")
    
    # Initialize the vector database collection
    vector_db.initialize_collection()
    
    # Get all chapters from the database
    chapters = get_all_chapters()
    print(f"Found {len(chapters)} chapters to process")
    
    if not chapters:
        print("No chapters found in the database. Please ensure chapters are loaded.")
        return
    
    # Process each chapter
    results = []
    for chapter in chapters:
        try:
            result = process_chapter_content(
                chapter_id=chapter['id'],
                title=chapter['title'],
                content=chapter['content']
            )
            results.append(result)
        except Exception as e:
            print(f"Error processing chapter {chapter['id']}: {str(e)}")
            results.append({
                "chapter_id": chapter['id'],
                "chapter_title": chapter['title'],
                "status": "error",
                "error": str(e)
            })
    
    # Print summary
    print("\nProcessing complete!")
    print(f"Processed {len([r for r in results if r['status'] == 'success'])} chapters successfully")
    print(f"Failed to process {len([r for r in results if r['status'] == 'error'])} chapters")
    
    # Print detailed results
    for result in results:
        status = "✓" if result['status'] == 'success' else "✗"
        print(f"{status} Chapter {result['chapter_id']}: {result['chapter_title']} - {result['status']}")


def load_chapters_from_docs(docs_dir: str):
    """
    Load chapters from the docs directory into the database
    This function will read all markdown files in subdirectories of docs
    and create chapter entries in the database.
    """
    db = SessionLocal()
    try:
        # Get or create modules (assuming we have modules for ROS2, Gazebo-Unity, etc.)
        from models.module import Module
        modules = db.query(Module).all()
        
        # If no modules exist, create some defaults
        if not modules:
            default_modules = [
                Module(title="ROS 2", description="Robot Operating System 2 fundamentals"),
                Module(title="Gazebo & Unity", description="Simulation environments"),
                Module(title="NVIDIA Isaac", description="AI framework for robotics"),
                Module(title="Vision-Language-Action", description="VLA models for robotics")
            ]
            
            for module in default_modules:
                db.add(module)
            db.commit()
            
            modules = db.query(Module).all()
        
        # Convert modules to a mapping for easy lookup
        module_map = {m.title.lower(): m.id for m in modules}
        
        # Process each subdirectory in docs
        docs_path = Path(docs_dir)
        for subdir in docs_path.iterdir():
            if subdir.is_dir():
                # Determine module based on directory name
                module_id = 1  # Default to first module
                for title, id in module_map.items():
                    if title.replace(" ", "-").lower() in subdir.name.lower():
                        module_id = id
                        break
                
                # Process each markdown file in the subdirectory
                for md_file in subdir.glob("*.md"):
                    try:
                        with open(md_file, 'r', encoding='utf-8') as f:
                            content = f.read()
                        
                        # Extract title from the first heading in the file
                        title = md_file.stem  # Use filename as title if no heading found
                        lines = content.split('\n')
                        for line in lines:
                            if line.startswith('# '):
                                title = line[2:].strip()
                                break
                        
                        # Check if chapter already exists
                        existing_chapter = db.query(Chapter).filter(
                            Chapter.title == title,
                            Chapter.module_id == module_id
                        ).first()
                        
                        if existing_chapter:
                            print(f"Updating existing chapter: {title}")
                            existing_chapter.content = content
                        else:
                            print(f"Creating new chapter: {title}")
                            chapter = Chapter(
                                title=title,
                                module_id=module_id,
                                content=content
                            )
                            db.add(chapter)
                        
                    except Exception as e:
                        print(f"Error processing {md_file}: {str(e)}")
        
        db.commit()
        print(f"Loaded chapters from {docs_dir}")
        
    finally:
        db.close()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Upload textbook chapters to Qdrant for RAG')
    parser.add_argument('--docs-dir', type=str, 
                       default='../frontend/docs',
                       help='Directory containing chapter markdown files')
    parser.add_argument('--load-docs', action='store_true',
                       help='Load chapters from docs directory before processing')
    
    args = parser.parse_args()
    
    if args.load_docs:
        print(f"Loading chapters from {args.docs_dir}")
        load_chapters_from_docs(args.docs_dir)
    
    main()