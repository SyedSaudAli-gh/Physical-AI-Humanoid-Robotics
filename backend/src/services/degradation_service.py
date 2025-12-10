# Graceful degradation service for handling external service unavailability
import logging
from typing import Dict, Any, Callable, Optional
from functools import wraps
import requests
from requests.exceptions import RequestException, Timeout
import time

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ServiceStatus:
    """
    Track the status of external services
    """
    def __init__(self):
        self.status = {
            'cohere': {'available': True, 'last_check': None, 'failure_count': 0},
            'qdrant': {'available': True, 'last_check': None, 'failure_count': 0},
            'fallback_active': False
        }
        self.max_failures = 3  # Max failures before marking service as unavailable
        self.recovery_timeout = 300  # 5 minutes before retesting service
    
    def record_failure(self, service_name: str):
        """
        Record a failure for a service
        """
        if service_name in self.status:
            self.status[service_name]['failure_count'] += 1
            self.status[service_name]['last_check'] = time.time()
            
            # Mark as unavailable if too many failures
            if self.status[service_name]['failure_count'] >= self.max_failures:
                self.status[service_name]['available'] = False
                logger.warning(f"{service_name} marked as unavailable due to repeated failures")
                
                # Activate fallback if all critical services are down
                if not self.status['cohere']['available'] and not self.status['qdrant']['available']:
                    self.status['fallback_active'] = True
    
    def record_success(self, service_name: str):
        """
        Record a success for a service
        """
        if service_name in self.status:
            self.status[service_name]['failure_count'] = 0
            self.status[service_name]['available'] = True
            self.status[service_name]['last_check'] = time.time()
            
            # Check if we can deactivate fallback
            if self.status['fallback_active']:
                if self.status['cohere']['available'] and self.status['qdrant']['available']:
                    self.status['fallback_active'] = False
                    logger.info("Fallback deactivated, all services available")
    
    def is_available(self, service_name: str) -> bool:
        """
        Check if a service is available
        """
        if service_name not in self.status:
            return True  # Assume available if not tracked
            
        service_info = self.status[service_name]
        
        # If marked as unavailable, check if it's time to retry
        if not service_info['available']:
            time_since_check = time.time() - service_info['last_check'] if service_info['last_check'] else float('inf')
            if time_since_check >= self.recovery_timeout:
                logger.info(f"Retrying {service_name} after timeout")
                return True  # Allow retry
        
        return service_info['available']

# Global service status tracker
service_status = ServiceStatus()

def check_service_availability(service_name: str, timeout: float = 5.0) -> bool:
    """
    Check if external service is actually available by pinging it
    """
    try:
        if service_name == 'cohere':
            # Try to make a simple API call to Cohere
            import os
            api_key = os.getenv("COHERE_API_KEY")
            if not api_key:
                return False
                
            headers = {"Authorization": f"Bearer {api_key}"}
            # Use a simple API endpoint to check availability
            response = requests.get(
                "https://api.cohere.ai/v1/check-api-key", 
                headers=headers, 
                timeout=timeout
            )
            return response.status_code == 200
            
        elif service_name == 'qdrant':
            # Try to make a simple API call to Qdrant
            import os
            qdrant_url = os.getenv("QDRANT_URL")
            if not qdrant_url:
                return False
                
            response = requests.get(f"{qdrant_url}/collections", timeout=timeout)
            return response.status_code in [200, 401, 403]  # 401/403 still means service is reachable
            
    except RequestException as e:
        logger.error(f"Error checking {service_name} availability: {str(e)}")
        return False
    except Exception as e:
        logger.error(f"Unexpected error checking {service_name} availability: {str(e)}")
        return False
    
    return True

def with_graceful_degradation(service_name: str, fallback_response: Optional[Dict[str, Any]] = None):
    """
    Decorator to add graceful degradation to functions that use external services
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            # Check if service is marked as available
            if service_status.is_available(service_name):
                try:
                    # If it's time to check again, verify actual availability
                    service_info = service_status.status[service_name]
                    time_since_check = time.time() - service_info['last_check'] if service_info['last_check'] else float('inf')
                    
                    if time_since_check >= service_status.recovery_timeout:
                        if not check_service_availability(service_name):
                            service_status.record_failure(service_name)
                            
                            # Use fallback if available
                            if fallback_response is not None:
                                logger.warning(f"Using fallback response for {service_name}")
                                return fallback_response
                            else:
                                # Return error response
                                return {
                                    'error': f'{service_name} service is temporarily unavailable',
                                    'fallback': True
                                }
                
                    # Call the original function
                    result = func(*args, **kwargs)
                    service_status.record_success(service_name)
                    return result
                except RequestException as e:
                    logger.error(f"Request error with {service_name}: {str(e)}")
                    service_status.record_failure(service_name)
                    
                    # Return fallback if available
                    if fallback_response is not None:
                        logger.warning(f"Using fallback response for {service_name}")
                        return fallback_response
                    else:
                        return {
                            'error': f'{service_name} service is temporarily unavailable',
                            'fallback': True
                        }
                except Exception as e:
                    logger.error(f"Unexpected error with {service_name}: {str(e)}")
                    service_status.record_failure(service_name)
                    
                    # Return fallback if available
                    if fallback_response is not None:
                        logger.warning(f"Using fallback response for {service_name}")
                        return fallback_response
                    else:
                        return {
                            'error': f'{service_name} service is temporarily unavailable',
                            'fallback': True
                        }
            else:
                # Service is marked as unavailable, return fallback or error
                if fallback_response is not None:
                    logger.warning(f"Using fallback response for {service_name} (marked unavailable)")
                    return fallback_response
                else:
                    return {
                        'error': f'{service_name} service is temporarily unavailable',
                        'fallback': True
                    }
        
        return wrapper
    return decorator

# Example usage functions
def get_embedding_with_fallback(text: str) -> Dict[str, Any]:
    """
    Example function that gets embeddings with graceful fallback
    """
    fallback_embedding = {'embedding': [0.1] * 1024, 'service': 'fallback'}
    
    @with_graceful_degradation('cohere', fallback_embedding)
    def _get_actual_embedding(text: str):
        # Actual Cohere embedding call would go here
        # For example purposes, we'll simulate it
        # from ..rag.cohere_embedder import embedding_service
        # return embedding_service.embed_text(text)
        pass
    
    return _get_actual_embedding(text)

def search_vector_db_with_fallback(query_embedding: list, limit: int = 5) -> Dict[str, Any]:
    """
    Example function that searches vector DB with graceful fallback
    """
    fallback_result = {'results': [], 'service': 'fallback', 'message': 'Using cached/local results'}
    
    @with_graceful_degradation('qdrant', fallback_result)
    def _search_actual_db(query_embedding: list, limit: int = 5):
        # Actual Qdrant search would go here
        # For example purposes, we'll simulate it
        # from ..rag.vector_db import vector_db
        # return vector_db.search_similar(query_embedding, limit)
        pass
    
    return _search_actual_db(query_embedding, limit)

if __name__ == "__main__":
    print("Service degradation handler initialized")
    print(f"Cohere status: {service_status.status['cohere']}")
    print(f"Qdrant status: {service_status.status['qdrant']}")