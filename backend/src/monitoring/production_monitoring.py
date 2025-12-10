"""
Monitoring and logging for production deployment of the Physical AI & Humanoid Robotics Textbook
"""
import logging
import logging.handlers
import os
from datetime import datetime
from typing import Dict, Any, Optional
import json
import traceback
from pathlib import Path

class ProductionLogger:
    """
    Production-ready logging system with monitoring capabilities
    """
    
    def __init__(self, app_name: str = "textbook_api", log_level: str = "INFO"):
        self.app_name = app_name
        self.log_level = getattr(logging, log_level.upper())
        self.logger = self.setup_logger()
    
    def setup_logger(self) -> logging.Logger:
        """
        Set up the production logger with file and console handlers
        """
        logger = logging.getLogger(self.app_name)
        logger.setLevel(self.log_level)
        
        # Prevent adding handlers multiple times
        if logger.handlers:
            return logger
        
        # Create logs directory if it doesn't exist
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)
        
        # Formatter for logs
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # File handler for detailed logs
        file_handler = logging.handlers.RotatingFileHandler(
            f"logs/{self.app_name}.log",
            maxBytes=10*1024*1024,  # 10MB
            backupCount=5
        )
        file_handler.setLevel(self.log_level)
        file_handler.setFormatter(formatter)
        
        # Console handler for important messages
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)
        console_handler.setFormatter(formatter)
        
        # Add handlers to logger
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def log_request(self, request_info: Dict[str, Any]):
        """
        Log incoming requests
        """
        self.logger.info(
            f"REQUEST: {request_info.get('method')} {request_info.get('path')} "
            f"FROM {request_info.get('client_ip', 'unknown')} "
            f"WITH USERAGENT: {request_info.get('user_agent', 'unknown')}"
        )
    
    def log_response(self, response_info: Dict[str, Any]):
        """
        Log outgoing responses
        """
        self.logger.info(
            f"RESPONSE: {response_info.get('status_code', 0)} "
            f"FOR {response_info.get('path', 'unknown')} "
            f"TOOK {response_info.get('response_time', 0):.4f}s"
        )
    
    def log_error(self, error_info: Dict[str, Any]):
        """
        Log errors with stack trace
        """
        self.logger.error(
            f"ERROR: {error_info.get('error_type', 'Unknown Error')} - "
            f"{error_info.get('error_message', 'No message')} - "
            f"PATH: {error_info.get('path', 'unknown')}",
            exc_info=True
        )
    
    def log_performance_metric(self, metric_name: str, value: float, unit: str = "seconds"):
        """
        Log performance metrics
        """
        self.logger.info(f"METRIC: {metric_name} = {value} {unit}")

class Monitor:
    """
    Application monitoring system
    """
    
    def __init__(self):
        self.logger = ProductionLogger()
        self.metrics = {}
        self.health_check_results = {}
    
    def start_monitoring(self):
        """
        Start application monitoring
        """
        self.logger.logger.info("Starting application monitoring...")
        # In a real implementation, you might connect to external monitoring services
        # like Prometheus, Datadog, or similar
    
    def log_api_call(self, endpoint: str, method: str, response_time: float, status_code: int):
        """
        Log API call metrics
        """
        self.logger.log_performance_metric(
            f"api_call_duration_{endpoint.replace('/', '_')}", 
            response_time
        )
        
        # Track metrics
        metric_key = f"{method}_{endpoint}"
        if metric_key not in self.metrics:
            self.metrics[metric_key] = []
        self.metrics[metric_key].append({
            'timestamp': datetime.utcnow().isoformat(),
            'response_time': response_time,
            'status_code': status_code
        })
    
    def log_db_operation(self, operation: str, duration: float, success: bool = True):
        """
        Log database operation
        """
        self.logger.log_performance_metric(
            f"db_operation_{operation}", 
            duration
        )
        
        status = "SUCCESS" if success else "FAILED"
        self.logger.logger.info(f"DB OPERATION: {operation} - {status} - {duration:.4f}s")
    
    def log_cache_operation(self, operation: str, duration: float, hit: bool = None):
        """
        Log cache operation
        """
        self.logger.log_performance_metric(
            f"cache_operation_{operation}", 
            duration
        )
        
        if hit is not None:
            hit_status = "HIT" if hit else "MISS"
            self.logger.logger.info(f"CACHE OPERATION: {operation} - {hit_status} - {duration:.4f}s")
        else:
            self.logger.logger.info(f"CACHE OPERATION: {operation} - {duration:.4f}s")
    
    def health_check(self) -> Dict[str, Any]:
        """
        Perform health check on the application
        """
        checks = {
            "timestamp": datetime.utcnow().isoformat(),
            "status": "healthy",
            "checks": {
                "database": self.check_database(),
                "external_apis": self.check_external_apis(),
                "disk_space": self.check_disk_space(),
                "memory_usage": self.check_memory_usage()
            }
        }
        
        # Determine overall status
        for check_name, result in checks["checks"].items():
            if not result["healthy"]:
                checks["status"] = "unhealthy"
                break
        
        self.health_check_results = checks
        self.logger.logger.info(f"Health check: {checks['status']}")
        return checks
    
    def check_database(self) -> Dict[str, Any]:
        """
        Check database connectivity
        """
        # In a real implementation, you'd actually test the database connection
        try:
            # Simulate database check
            is_healthy = True  # Placeholder - in real implementation, test actual connection
            return {
                "healthy": is_healthy,
                "response_time_ms": 10,  # Placeholder
                "details": "Database connection OK" if is_healthy else "Connection failed"
            }
        except Exception as e:
            return {
                "healthy": False,
                "response_time_ms": 0,
                "details": f"Database error: {str(e)}"
            }
    
    def check_external_apis(self) -> Dict[str, Any]:
        """
        Check external API connections (Cohere, Qdrant, etc.)
        """
        # In a real implementation, you'd actually test the external service connections
        try:
            # Simulate external API checks
            services = {
                "cohere": True,  # Placeholder
                "qdrant": True,  # Placeholder
                "better_auth": True  # Placeholder
            }
            
            all_healthy = all(services.values())
            return {
                "healthy": all_healthy,
                "services": services,
                "details": "All external services accessible" if all_healthy else "Some services unreachable"
            }
        except Exception as e:
            return {
                "healthy": False,
                "services": {},
                "details": f"External API check error: {str(e)}"
            }
    
    def check_disk_space(self) -> Dict[str, Any]:
        """
        Check disk space
        """
        try:
            # Get disk usage (in bytes)
            total, used, free = self._get_disk_usage()
            
            # Calculate usage percentage
            usage_percent = (used / total) * 100
            is_healthy = usage_percent < 80  # Healthy if less than 80% used
            
            return {
                "healthy": is_healthy,
                "usage_percent": round(usage_percent, 2),
                "free_space_gb": round(free / (1024**3), 2),
                "total_space_gb": round(total / (1024**3), 2),
                "details": f'Disk usage: {usage_percent:.2f}% ({round(free / (1024**3), 2)}GB free)'
            }
        except Exception as e:
            return {
                "healthy": False,
                "usage_percent": 0,
                "details": f"Disk space check error: {str(e)}"
            }
    
    def check_memory_usage(self) -> Dict[str, Any]:
        """
        Check memory usage
        """
        try:
            # Simulate memory usage check
            # In a real implementation, you'd use psutil or similar to get actual memory usage
            import psutil
            memory = psutil.virtual_memory()
            
            usage_percent = memory.percent
            is_healthy = usage_percent < 80  # Healthy if less than 80% used
            
            return {
                "healthy": is_healthy,
                "usage_percent": usage_percent,
                "available_mb": round(memory.available / (1024**2), 2),
                "total_mb": round(memory.total / (1024**2), 2),
                "details": f'Memory usage: {usage_percent}% ({round(memory.available / (1024**2), 2)}MB available)'
            }
        except Exception as e:
            return {
                "healthy": False,
                "usage_percent": 0,
                "details": f"Memory usage check error: {str(e)}"
            }
    
    def _get_disk_usage(self):
        """
        Helper to get disk usage statistics
        """
        # In a real implementation, you'd use shutil.disk_usage or similar
        import shutil
        total, used, free = shutil.disk_usage(".")
        return total, used, free

# Middleware to integrate with FastAPI
from fastapi import Request
from time import time

async def monitoring_middleware(request: Request, call_next):
    """
    FastAPI middleware to monitor requests
    """
    start_time = time()
    monitor = Monitor()
    
    # Log the incoming request
    monitor.logger.log_request({
        "method": request.method,
        "path": str(request.url.path),
        "client_ip": request.client.host,
        "user_agent": request.headers.get("user-agent", "unknown")
    })
    
    try:
        response = await call_next(request)
        
        # Calculate response time
        response_time = time() - start_time
        
        # Log the response
        monitor.logger.log_response({
            "status_code": response.status_code,
            "path": str(request.url.path),
            "response_time": response_time
        })
        
        # Log API call for metrics
        monitor.log_api_call(
            endpoint=str(request.url.path),
            method=request.method,
            response_time=response_time,
            status_code=response.status_code
        )
        
        return response
    
    except Exception as e:
        # Calculate response time even if error
        response_time = time() - start_time
        monitor.logger.log_error({
            "error_type": type(e).__name__,
            "error_message": str(e),
            "path": str(request.url.path),
            "response_time": response_time
        })
        
        # Re-raise the exception
        raise

# Example usage
if __name__ == "__main__":
    # Initialize the monitoring system
    monitor = Monitor()
    monitor.start_monitoring()
    
    # Perform a sample health check
    health_status = monitor.health_check()
    print(f"Health Check Result: {health_status['status']}")
    
    # Log a sample API call
    monitor.log_api_call("/api/modules", "GET", 0.123, 200)
    
    # Log a sample database operation
    monitor.log_db_operation("select_modules", 0.045, True)
    
    print("Production monitoring initialized and running")
    print("Logging and monitoring components ready for deployment")