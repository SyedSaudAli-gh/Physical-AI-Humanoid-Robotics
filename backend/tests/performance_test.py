"""
Performance testing script for the Physical AI & Humanoid Robotics Textbook project

This script tests the application's performance under load with 100-500 simulated concurrent users
"""
import asyncio
import aiohttp
import time
from typing import List, Dict, Any
import statistics
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PerformanceTester:
    """
    Class to perform performance testing on the textbook application
    """
    
    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url
        self.session = None
    
    async def __aenter__(self):
        """
        Initialize the aiohttp session
        """
        self.session = aiohttp.ClientSession()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """
        Close the aiohttp session
        """
        if self.session:
            await self.session.close()
    
    async def test_endpoint(self, endpoint: str, method: str = "GET", payload: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Test a specific endpoint and return timing information
        """
        start_time = time.time()
        status_code = 0
        response_text = ""
        
        try:
            url = f"{self.base_url}{endpoint}"
            if method.upper() == "GET":
                async with self.session.get(url) as response:
                    response_text = await response.text()
                    status_code = response.status
            elif method.upper() == "POST" and payload:
                async with self.session.post(url, json=payload) as response:
                    response_text = await response.text()
                    status_code = response.status
        except Exception as e:
            logger.error(f"Error testing {url}: {str(e)}")
            return {
                "endpoint": endpoint,
                "method": method,
                "status_code": 0,
                "response_time": time.time() - start_time,
                "error": str(e)
            }
        
        response_time = time.time() - start_time
        
        return {
            "endpoint": endpoint,
            "method": method,
            "status_code": status_code,
            "response_time": response_time,
            "content_length": len(response_text)
        }
    
    async def simulate_concurrent_users(self, endpoint: str, method: str = "GET", 
                                       payload: Dict[str, Any] = None, num_users: int = 100) -> List[Dict[str, Any]]:
        """
        Simulate multiple concurrent users accessing an endpoint
        """
        logger.info(f"Testing {num_users} concurrent users on {endpoint}")
        
        tasks = []
        for _ in range(num_users):
            task = asyncio.create_task(
                self.test_endpoint(endpoint, method, payload)
            )
            tasks.append(task)
        
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        # Filter out any exceptions
        valid_results = []
        for result in results:
            if isinstance(result, dict):
                valid_results.append(result)
        
        return valid_results
    
    def analyze_results(self, results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Analyze the performance test results
        """
        if not results:
            return {"error": "No valid results to analyze"}
        
        response_times = [r["response_time"] for r in results if r.get("response_time")]
        status_codes = [r["status_code"] for r in results if r.get("status_code")]
        
        analysis = {
            "total_requests": len(results),
            "successful_requests": sum(1 for r in results if 200 <= r.get("status_code", 0) < 300),
            "failed_requests": sum(1 for r in results if not (200 <= r.get("status_code", 0) < 300)),
            "response_time_avg": statistics.mean(response_times) if response_times else 0,
            "response_time_median": statistics.median(response_times) if response_times else 0,
            "response_time_max": max(response_times) if response_times else 0,
            "response_time_min": min(response_times) if response_times else 0,
            "status_codes_distribution": {code: status_codes.count(code) for code in set(status_codes)}
        }
        
        if len(response_times) > 1:
            analysis["response_time_std_dev"] = statistics.stdev(response_times)
        
        # Calculate success rate
        analysis["success_rate"] = analysis["successful_requests"] / analysis["total_requests"] * 100
        
        # Check if meets performance requirements (load times under 5s)
        analysis["meets_performance_requirements"] = analysis["response_time_avg"] < 5.0
        
        return analysis
    
    async def run_comprehensive_test(self) -> Dict[str, Any]:
        """
        Run comprehensive performance tests on multiple endpoints
        """
        logger.info("Starting comprehensive performance testing...")
        
        # Define test scenarios
        test_scenarios = [
            {"endpoint": "/", "method": "GET"},
            {"endpoint": "/api/modules", "method": "GET"},
            {"endpoint": "/api/chapters", "method": "GET"},
            {
                "endpoint": "/api/chat/query", 
                "method": "POST", 
                "payload": {"query": "What is ROS 2?"}
            }
        ]
        
        results = {}
        
        for scenario in test_scenarios:
            # Test with different numbers of concurrent users
            for user_count in [10, 50, 100, 200, 300, 400, 500]:
                endpoint_results = await self.simulate_concurrent_users(
                    scenario["endpoint"], 
                    scenario.get("method", "GET"), 
                    scenario.get("payload"),
                    user_count
                )
                
                analysis = self.analyze_results(endpoint_results)
                key = f"{scenario['endpoint']}_{user_count}_users"
                results[key] = {
                    "scenario": scenario,
                    "user_count": user_count,
                    "analysis": analysis
                }
                
                logger.info(
                    f"Completed test for {key}: "
                    f"Avg response time: {analysis['response_time_avg']:.3f}s, "
                    f"Success rate: {analysis['success_rate']:.2f}%"
                )
        
        return results

async def run_performance_tests():
    """
    Main function to run performance tests
    """
    async with PerformanceTester() as tester:
        results = await tester.run_comprehensive_test()
        
        # Print summary
        print("\n" + "="*80)
        print("PERFORMANCE TEST SUMMARY")
        print("="*80)
        
        for test_name, data in results.items():
            analysis = data["analysis"]
            print(f"\nTest: {test_name}")
            print(f"  Successful Requests: {analysis['successful_requests']}/{analysis['total_requests']} ({analysis['success_rate']:.2f}%)")
            print(f"  Avg Response Time: {analysis['response_time_avg']:.3f}s")
            print(f"  Median Response Time: {analysis['response_time_median']:.3f}s")
            print(f"  Max Response Time: {analysis['response_time_max']:.3f}s")
            print(f"  Min Response Time: {analysis['response_time_min']:.3f}s")
            print(f"  Meets Performance Requirements: {analysis['meets_performance_requirements']}")
        
        # Overall summary
        all_results = [data["analysis"] for data in results.values()]
        avg_response_time = statistics.mean([a["response_time_avg"] for a in all_results])
        avg_success_rate = statistics.mean([a["success_rate"] for a in all_results])
        
        print("\n" + "="*80)
        print("OVERALL PERFORMANCE SUMMARY")
        print("="*80)
        print(f"Average Response Time Across All Tests: {avg_response_time:.3f}s")
        print(f"Average Success Rate Across All Tests: {avg_success_rate:.2f}%")
        
        # Check if system meets requirements
        meets_requirements = (
            avg_response_time < 5.0 and
            avg_success_rate >= 95.0
        )
        
        print(f"Meets Performance Requirements: {meets_requirements}")
        
        if meets_requirements:
            print("✅ System meets performance requirements!")
        else:
            print("❌ System does not meet performance requirements")
        
        return results

if __name__ == "__main__":
    print("Starting performance testing for Physical AI & Humanoid Robotics Textbook...")
    print("Testing with 100-500 simulated concurrent users...")
    
    # Run the performance tests
    asyncio.run(run_performance_tests())
    
    print("\nPerformance testing completed!")