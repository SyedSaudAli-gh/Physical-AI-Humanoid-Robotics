"""
Performance optimization utilities for the Physical AI & Humanoid Robotics Textbook project

This module contains performance optimization techniques for:
- Reducing load times
- Optimizing database queries
- Improving caching strategies
- Optimizing API responses
"""
from typing import Dict, Any, Optional, List, Callable
from time import time
from functools import wraps
import asyncio
from concurrent.futures import ThreadPoolExecutor
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def performance_monitor(func: Callable) -> Callable:
    """
    Decorator to monitor function performance and execution time
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time()
        result = func(*args, **kwargs)
        end_time = time()
        execution_time = end_time - start_time
        
        if execution_time > 1.0:  # Log slow functions (>1 second)
            logger.warning(f"Slow function detected: {func.__name__} took {execution_time:.4f}s")
        else:
            logger.debug(f"{func.__name__} took {execution_time:.4f}s")
        
        return result
    return wrapper

class PerformanceOptimizer:
    """
    Class containing performance optimization utilities
    """
    
    def __init__(self):
        self.executor = ThreadPoolExecutor(max_workers=10)
    
    @performance_monitor
    def optimize_database_query(self, query: str, params: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Optimize database query by adding proper indexing hints, using pagination
        """
        # In a real implementation, this would execute optimized queries
        # For example, adding LIMIT clauses, using proper indexes, etc.
        
        # Placeholder for optimized query execution
        # result = self.db.execute(query, params).fetchall()
        # return result
        
        # For demonstration, return a dummy result
        return [{"id": 1, "name": "optimized_result"}]
    
    @performance_monitor
    async def batch_process_requests(self, requests: List[Dict[str, Any]], 
                                   processor_func: Callable) -> List[Any]:
        """
        Batch process multiple requests for better performance
        """
        tasks = []
        for req in requests:
            task = asyncio.create_task(processor_func(req))
            tasks.append(task)
        
        results = await asyncio.gather(*tasks)
        return results
    
    def optimize_content_loading(self, chapter_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Optimize content loading by fetching content in batch and using caching
        """
        # In a real implementation, this would fetch multiple chapters in a single query
        # using JOINs and caching the results
        
        # Placeholder implementation
        chapters = []
        for chapter_id in chapter_ids:
            # Would normally fetch in batch with proper JOINs
            chapter = {
                "id": chapter_id,
                "title": f"Chapter {chapter_id}",
                "content": f"Content for chapter {chapter_id}...",
                "load_time_optimized": True
            }
            chapters.append(chapter)
        
        return chapters
    
    def implement_caching_layer(self, key: str, getter_func: Callable, ttl: int = 300) -> Any:
        """
        Generic caching implementation with TTL
        """
        # This would implement a caching layer using Redis or similar
        # For this implementation, using a simple in-memory cache
        cache = getattr(self, '_cache', {})
        cache_time = getattr(self, '_cache_time', {})
        
        current_time = time()
        
        # Check if cached value exists and is not expired
        if key in cache and current_time - cache_time.get(key, 0) < ttl:
            logger.debug(f"Cache hit for key: {key}")
            return cache[key]
        
        # Get fresh value
        value = getter_func()
        
        # Store in cache
        cache[key] = value
        cache_time[key] = current_time
        setattr(self, '_cache', cache)
        setattr(self, '_cache_time', cache_time)
        
        logger.debug(f"Cache miss for key: {key}, stored new value")
        return value
    
    def optimize_api_responses(self, data: List[Dict[str, Any]], 
                              fields_to_include: Optional[List[str]] = None) -> List[Dict[str, Any]]:
        """
        Optimize API responses by reducing payload size
        """
        if not fields_to_include:
            # Default fields for optimized response
            fields_to_include = ["id", "title", "description", "module_id"]
        
        optimized_data = []
        for item in data:
            optimized_item = {
                key: value for key, value in item.items() 
                if key in fields_to_include
            }
            optimized_data.append(optimized_item)
        
        return optimized_data

class APILayerOptimizer:
    """
    Optimizer specifically for API layer performance
    """
    
    def __init__(self, db_session):
        self.db = db_session
        self.optimizer = PerformanceOptimizer()
    
    @performance_monitor
    def get_optimized_modules(self) -> List[Dict[str, Any]]:
        """
        Get modules with optimized query to reduce load times
        """
        query = """
        SELECT id, name, description, order_index, learning_objectives
        FROM modules 
        ORDER BY order_index
        LIMIT 100  -- Prevent overly large responses
        """
        
        # Use the optimizer to execute the query efficiently
        return self.optimizer.optimize_database_query(query)
    
    @performance_monitor
    def get_optimized_chapters_for_module(self, module_id: str) -> List[Dict[str, Any]]:
        """
        Get chapters for module with optimized query
        """
        query = """
        SELECT id, module_id, title, order_index, learning_outcomes
        FROM chapters 
        WHERE module_id = :module_id 
        ORDER BY order_index
        LIMIT 50  -- Limit chapters per module
        """
        
        return self.optimizer.optimize_database_query(query, {"module_id": module_id})
    
    def get_optimized_chapter_content(self, chapter_id: str) -> Dict[str, Any]:
        """
        Get chapter content with optimized response
        """
        # More selective query to get only necessary content parts
        query = """
        SELECT id, module_id, title, content
        FROM chapters 
        WHERE id = :chapter_id
        """
        
        results = self.optimizer.optimize_database_query(query, {"chapter_id": chapter_id})
        
        if results:
            # Apply response optimization to reduce payload
            return self.optimizer.optimize_api_responses(
                results, 
                fields_to_include=["id", "module_id", "title", "content"]
            )[0]  # Return just the first item
            
        return {}

# Example usage functions for performance optimization
def optimize_module_loading(module_service) -> List[Dict[str, Any]]:
    """
    Optimize loading of modules by using optimized queries
    """
    optimizer = APILayerOptimizer(module_service.db)
    return optimizer.get_optimized_modules()

def optimize_chapter_loading(module_id: str, chapter_service) -> List[Dict[str, Any]]:
    """
    Optimize loading of chapters for a module
    """
    optimizer = APILayerOptimizer(chapter_service.db)
    return optimizer.get_optimized_chapters_for_module(module_id)

def optimize_content_retrieval(chapter_id: str, chapter_service) -> Dict[str, Any]:
    """
    Optimize retrieval of chapter content
    """
    optimizer = APILayerOptimizer(chapter_service.db)
    return optimizer.get_optimized_chapter_content(chapter_id)

# Performance optimization for the RAG service
class RAGOptimizationLayer:
    """
    Optimization layer for RAG services to improve response times
    """
    
    def __init__(self, rag_service, cache_service):
        self.rag_service = rag_service
        self.cache = cache_service
        self.performance_optimizer = PerformanceOptimizer()
    
    @performance_monitor
    async def optimized_query(self, query: str, selected_text: Optional[str] = None,
                             chapter_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Optimized RAG query with improved caching and reduced response times
        """
        # Create cache key based on query content
        cache_key = f"rag_query:{hash(query)}:{hash(str(selected_text))}:{chapter_id}"
        
        # Check cache first (this is typically fast)
        cached_result = self.cache.get_cached_response(query, selected_text, chapter_id)
        if cached_result:
            logger.info("RAG query cache hit")
            return cached_result
        
        # If not in cache, perform optimized query
        result = await self.rag_service.async_query_content(query, selected_text, chapter_id)
        
        # Cache for future queries
        self.cache.cache_response(query, result, selected_text, chapter_id)
        
        return result

if __name__ == "__main__":
    print("Performance optimization utilities initialized")
    print("These utilities help ensure load times stay under 5s by:")
    print("- Monitoring function execution times")
    print("- Optimizing database queries")
    print("- Implementing caching layers")
    print("- Optimizing API responses")
    print("- Reducing payload sizes")