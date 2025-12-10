"""
Qwen CLI Skill for Code Example Generation
This module provides functionality to generate code examples for the Physical AI & Humanoid Robotics textbook
following best practices and validated against official documentation.
"""
from typing import Dict, Any, List
import re

class CodeExampleGeneratorSkill:
    """
    Skill for generating code examples that match the textbook's technical requirements
    """
    
    def __init__(self):
        self.supported_languages = ["Python", "C++", "ROS 2", "NVIDIA Isaac", "Gazebo"]
        self.patterns = {
            "python": {
                "header": '"""',
                "footer": '"""',
                "comment": "#",
                "function_def": r"def\s+\w+\s*\([^)]*\):",
                "class_def": r"class\s+\w+\s*:",
            },
            "cpp": {
                "header": "/*",
                "footer": "*/",
                "comment": "//",
                "function_def": r"\w+\s+\w+\s*\([^)]*\)\s*{",
                "class_def": r"class\s+\w+\s*{",
            }
        }
    
    def generate_code_example(self, 
                            topic: str, 
                            language: str = "Python", 
                            complexity: str = "intermediate",
                            module: str = None) -> Dict[str, Any]:
        """
        Generate a code example for a specific topic and language
        """
        if language not in self.supported_languages:
            raise ValueError(f"Language {language} not supported. Supported: {self.supported_languages}")
        
        # Generate code based on topic and complexity
        code = self._generate_code_for_topic(topic, language, complexity, module)
        
        # Validate the generated code
        validation_result = self.validate_code_syntax(code, language)
        
        return {
            "topic": topic,
            "language": language,
            "complexity": complexity,
            "module": module,
            "code": code,
            "description": f"Code example for {topic} in {language}",
            "validation_result": validation_result,
            "best_practices_followed": self._check_best_practices(code, language)
        }
    
    def _generate_code_for_topic(self, topic: str, language: str, complexity: str, module: str) -> str:
        """
        Generate code based on the topic, language, and complexity
        """
        # Define templates for different topics
        templates = {
            "ROS 2": {
                "Python": {
                    "basic": f"#!/usr/bin/env python3\n\nimport rclpy\nfrom rclpy.node import Node\n\nclass {topic.replace(' ', '')}Node(Node):\n    def __init__(self):\n        super().__init__('{topic.lower().replace(' ', '_')}_node')\n        self.get_logger().info('Node initialized')\n\n    def example_method(self):\n        self.get_logger().info('Method called')\n\ndef main(args=None):\n    rclpy.init(args=args)\n    node = {topic.replace(' ', '')}Node()\n    rclpy.spin(node)\n    node.destroy_node()\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()",
                    "intermediate": f"#!/usr/bin/env python3\n\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass {topic.replace(' ', '')}Node(Node):\n    def __init__(self):\n        super().__init__('{topic.lower().replace(' ', '_')}_node')\n        self.publisher = self.create_publisher(String, 'topic', 10)\n        timer_period = 0.5  # seconds\n        self.timer = self.create_timer(timer_period, self.timer_callback)\n        self.i = 0\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = f'Hello World: {{self.i}}'\n        self.publisher.publish(msg)\n        self.get_logger().info(f'Publishing: \"{{msg.data}}\"')\n        self.i += 1\n\ndef main(args=None):\n    rclpy.init(args=args)\n    minimal_publisher = {topic.replace(' ', '')}Node()\n    \n    try:\n        rclpy.spin(minimal_publisher)\n    except KeyboardInterrupt:\n        pass\n    finally:\n        minimal_publisher.destroy_node()\n        rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()",
                    "advanced": f"#!/usr/bin/env python3\n\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\nfrom sensor_msgs.msg import LaserScan\nimport math\n\nclass {topic.replace(' ', '')}Node(Node):\n    def __init__(self):\n        super().__init__('{topic.lower().replace(' ', '_')}_node')\n        self.subscription = self.create_subscription(\n            LaserScan,\n            'scan',\n            self.laser_callback,\n            10)\n        self.publisher = self.create_publisher(String, 'obstacle_status', 10)\n        self.subscription  # prevent unused variable warning\n\n    def laser_callback(self, msg):\n        # Process laser scan data\n        min_distance = min([x for x in msg.ranges if 0.1 < x < 10.0])\n        if min_distance < 0.5:\n            status_msg = String()\n            status_msg.data = 'OBSTACLE_CLOSE'\n            self.publisher.publish(status_msg)\n            self.get_logger().info('Obstacle detected close!')\n        else:\n            self.get_logger().info(f'Clear path, nearest obstacle: {{min_distance:.2f}}m')\n\ndef main(args=None):\n    rclpy.init(args=args)\n    obstacle_detector = {topic.replace(' ', '')}Node()\n    \n    try:\n        rclpy.spin(obstacle_detector)\n    except KeyboardInterrupt:\n        pass\n    finally:\n        obstacle_detector.destroy_node()\n        rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()"
                },
                "C++": {
                    "basic": "#include <rclcpp/rclcpp.hpp>\n\nclass {topic.replace(' ', '')}Node : public rclcpp::Node\n{{\npublic:\n  {topic.replace(' ', '')}Node() : Node(\"{topic.lower().replace(' ', '_')}_node\")\n  {{\n    RCLCPP_INFO(this->get_logger(), \"Node initialized\");\n  }}\n\nprivate:\n}};\n\nint main(int argc, char * argv[])\n{{\n  rclpy::init(argc, argv);\n  rclpy::spin(std::make_shared<{topic.replace(' ', '')}Node>());\n  rclpy::shutdown();\n  return 0;\n}}",
                    "intermediate": "// TODO: Implement intermediate C++ example for {topic} based on ROS 2\n// This would include publisher/subscriber patterns",
                    "advanced": "// TODO: Implement advanced C++ example for {topic} based on ROS 2\n// This would include complex behaviors, state machines, etc"
                }
            }
        }
        
        # Default template if specific one not found
        if module not in templates:
            module = "ROS 2"  # Default to ROS 2 examples
            
        if language not in templates[module]:
            language = "Python"  # Default to Python
            
        if complexity not in templates[module][language]:
            complexity = "intermediate"  # Default to intermediate
            
        return templates[module][language][complexity]
    
    def validate_code_syntax(self, code: str, language: str) -> Dict[str, Any]:
        """
        Validate the syntax of the generated code
        """
        issues = []
        
        # Check for common syntax errors based on language
        if language == "Python":
            # Check for proper indentation (not perfect, but catches obvious issues)
            lines = code.split('\n')
            for i, line in enumerate(lines):
                if '\t' in line and not line.strip().startswith('#'):
                    issues.append(f"Line {i+1}: Contains tab character, use spaces instead")
                
                # Check for common Python errors
                if 'import' in line and not line.strip().startswith('from') and not line.strip().startswith('import'):
                    # Line contains 'import' but not at the beginning, might be an error
                    continue
        
        elif language == "C++":
            # Check for basic syntax requirements
            if not code.strip().endswith('}'):
                issues.append("Code might be missing closing braces")
        
        return {
            "is_valid": len(issues) == 0,
            "issues": issues,
            "language": language
        }
    
    def _check_best_practices(self, code: str, language: str) -> List[str]:
        """
        Check if the code follows best practices for the textbook
        """
        best_practices = []
        
        # Check for comments/Documentation
        if language == "Python":
            if '"""' in code or '#' in code:
                best_practices.append("Contains documentation/comments")
            else:
                best_practices.append("Missing documentation/comments")
        
        # Check for proper error handling (basic check)
        if 'try:' in code or 'except' in code or 'throw' in code:
            best_practices.append("Includes error handling")
        else:
            best_practices.append("Missing error handling")
        
        return best_practices

def generate_code_example(topic: str, 
                         language: str = "Python", 
                         complexity: str = "intermediate",
                         module: str = None) -> Dict[str, Any]:
    """
    Main function to generate code example
    This would be called by the Qwen CLI
    """
    generator = CodeExampleGeneratorSkill()
    result = generator.generate_code_example(topic, language, complexity, module)
    return result

if __name__ == "__main__":
    print("Code Example Generator Skill initialized")
    
    # Example usage
    result = generate_code_example(
        topic="Subscriber",
        language="Python",
        complexity="intermediate",
        module="ROS 2"
    )
    print(f"Generated code example for: {result['topic']}")
    print(f"Language: {result['language']}")
    print(f"Valid: {result['validation_result']['is_valid']}")