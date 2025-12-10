import time
from typing import Dict, Any, Optional
from collections import OrderedDict

class SimpleCache:
    """
    Simple in-memory cache for RAG responses and translations
    """

    def __init__(self, max_size: int = 1000, ttl: int = 3600):
        """
        Initialize cache with max size and time-to-live (in seconds)
        """
        self.max_size = max_size
        self.ttl = ttl
        self.cache = OrderedDict()

    def get(self, key: str) -> Optional[Dict[str, Any]]:
        """
        Get value from cache if it exists and hasn't expired
        """
        if key in self.cache:
            value, timestamp = self.cache[key]

            # Check if expired
            if time.time() - timestamp > self.ttl:
                del self.cache[key]
                return None

            # Move to end to show it was recently accessed
            self.cache.move_to_end(key)
            return value

        return None

    def set(self, key: str, value: Dict[str, Any]):
        """
        Set value in cache with current timestamp
        """
        # Remove oldest item if at max capacity
        if len(self.cache) >= self.max_size:
            self.cache.popitem(last=False)

        self.cache[key] = (value, time.time())

    def clear(self):
        """
        Clear all items from cache
        """
        self.cache.clear()

# Global cache instances
rag_cache = SimpleCache(max_size=300, ttl=1800)  # 300 items, 30 minutes TTL for RAG
translation_cache = SimpleCache(max_size=200, ttl=3600)  # 200 items, 60 minutes TTL for translations

class CacheService:
    """
    Service for handling caching of RAG responses and translations
    """

    @staticmethod
    def get_cache_key(query: str, selected_text: str = None, chapter_id: str = None) -> str:
        """
        Generate a cache key from query and context (for RAG)
        """
        parts = [query.lower().strip()]
        if selected_text:
            parts.append(selected_text.lower().strip())
        if chapter_id:
            parts.append(chapter_id)

        # Create a simple hash-like key
        return "_rag_".join(parts[:3])[:255]  # Limit key length, prefix with _rag_

    @staticmethod
    def get_translation_cache_key(content: str, target_lang: str, source_lang: str = "en") -> str:
        """
        Generate a cache key for translation
        """
        content_hash = str(hash(content))[:10]  # Use content hash to avoid long keys
        key = f"trans_{content_hash}_{source_lang}_to_{target_lang}"
        return key[:255]  # Limit key length

    @staticmethod
    def get_cached_response(query: str, selected_text: str = None, chapter_id: str = None) -> Optional[Dict[str, Any]]:
        """
        Get RAG response from cache if available
        """
        key = CacheService.get_cache_key(query, selected_text, chapter_id)
        return rag_cache.get(key)

    @staticmethod
    def cache_response(query: str, response: Dict[str, Any], selected_text: str = None, chapter_id: str = None):
        """
        Cache a RAG response
        """
        key = CacheService.get_cache_key(query, selected_text, chapter_id)
        rag_cache.set(key, response)

    @staticmethod
    def get_cached_translation(content: str, target_lang: str, source_lang: str = "en") -> Optional[str]:
        """
        Get cached translation if available
        """
        key = CacheService.get_translation_cache_key(content, target_lang, source_lang)
        cached = translation_cache.get(key)
        return cached.get('translated_content') if cached else None

    @staticmethod
    def cache_translation(content: str, translated_content: str, target_lang: str, source_lang: str = "en"):
        """
        Cache a translation
        """
        key = CacheService.get_translation_cache_key(content, target_lang, source_lang)
        translation_cache.set(key, {
            'original_content': content,
            'translated_content': translated_content,
            'target_language': target_lang,
            'source_language': source_lang
        })

    @staticmethod
    def invalidate_cache():
        """
        Clear all caches
        """
        rag_cache.clear()
        translation_cache.clear()

# Example usage
if __name__ == "__main__":
    # Example usage
    query = "What are the core concepts of ROS 2?"
    response = {
        'query': query,
        'response': 'ROS 2 provides a communication infrastructure for robotics applications...',
        'sources': [{'chapter_id': 'ch1', 'relevance_score': 0.95}],
        'timestamp': '2023-01-01T00:00:00Z'
    }

    # Cache the response
    CacheService.cache_response(query, response)

    # Retrieve from cache
    cached = CacheService.get_cached_response(query)
    print(f"Cached response: {cached}")

    # Test translation caching
    original_content = "This is a test for translation caching"
    translated_content = "یہ ترجمہ کی کیش کے لئے ایک ٹیسٹ ہے"

    CacheService.cache_translation(original_content, translated_content, "ur")
    cached_translation = CacheService.get_cached_translation(original_content, "ur")
    print(f"Cached translation: {cached_translation}")