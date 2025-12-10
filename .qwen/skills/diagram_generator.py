"""
Qwen CLI Skill for Diagram Generation
This module provides functionality to generate diagrams for the Physical AI & Humanoid Robotics textbook
using Mermaid or other diagramming tools as appropriate for the content.
"""
from typing import Dict, Any, List
import re

class DiagramGeneratorSkill:
    """
    Skill for generating diagrams that illustrate concepts in the textbook
    """
    
    def __init__(self):
        self.supported_diagram_types = [
            "flowchart", "sequence", "class", "state", "er", "user-journey", "gantt", "pie"
        ]
    
    def generate_diagram(self, 
                        topic: str, 
                        diagram_type: str = "flowchart", 
                        description: str = None) -> Dict[str, Any]:
        """
        Generate a diagram for a specific topic
        """
        if diagram_type not in self.supported_diagram_types:
            diagram_type = "flowchart"  # default
        
        # Generate diagram code based on topic and type
        diagram_code = self._generate_diagram_code(topic, diagram_type, description)
        
        return {
            "topic": topic,
            "diagram_type": diagram_type,
            "description": description or f"Diagram for {topic}",
            "diagram_code": diagram_code,
            "supported_format": "mermaid",
            "usage_context": self._determine_usage_context(topic)
        }
    
    def _generate_diagram_code(self, topic: str, diagram_type: str, description: str) -> str:
        """
        Generate the actual diagram code based on the topic
        """
        # Normalize the topic for generating appropriate diagrams
        topic_lower = topic.lower()
        
        if diagram_type == "flowchart":
            return self._generate_flowchart(topic_lower)
        elif diagram_type == "sequence":
            return self._generate_sequence_diagram(topic_lower)
        elif diagram_type == "class":
            return self._generate_class_diagram(topic_lower)
        elif diagram_type == "state":
            return self._generate_state_diagram(topic_lower)
        elif diagram_type == "er":
            return self._generate_er_diagram(topic_lower)
        elif diagram_type == "user-journey":
            return self._generate_user_journey(topic_lower)
        elif diagram_type == "gantt":
            return self._generate_gantt_chart(topic_lower)
        elif diagram_type == "pie":
            return self._generate_pie_chart(topic_lower)
        else:
            # Default to flowchart
            return self._generate_flowchart(topic_lower)
    
    def _generate_flowchart(self, topic: str) -> str:
        """
        Generate a flowchart for the given topic
        """
        # Templates for different robotics-related topics
        templates = {
            "ros node": """graph TD
    A[Initialize Node] --> B[Create Publisher/Subscriber]
    B --> C[Define Callback Functions]
    C --> D[Spin Node]
    D --> E[Process Messages]
    E --> F[Execute Actions]
    F --> G[Continue Loop]
    G --> D
    D --> H{Shutdown?}
    H -->|Yes| I[Cleanup Resources]
    H -->|No| D""",
            "planning": """graph TD
    A[Perceive Environment] --> B[Identify Goal]
    B --> C[Generate Path Options]
    C --> D{Path Valid?}
    D -->|Yes| E[Select Optimal Path]
    D -->|No| C
    E --> F[Execute Movement]
    F --> G[Monitor Progress]
    G --> H{Goal Reached?}
    H -->|Yes| I[Task Complete]
    H -->|No| A""",
            "control": """graph TD
    A[Sensor Input] --> B[Process Data]
    B --> C[Apply Control Law]
    C --> D[Generate Command]
    D --> E[Actuator Output]
    E --> F[Physical Action]
    F --> A""",
            "perception": """graph TD
    A[Sensor Data] --> B[Preprocessing]
    B --> C[Feature Extraction]
    C --> D[Object Detection]
    D --> E{Object Found?}
    E -->|Yes| F[Object Classification]
    E -->|No| B
    F --> G[Localization]
    G --> H[Update World Model]
    H --> I[Output Processed Data]""",
            "default": f"""graph TD
    A[Start - {topic}] --> B[Process]
    B --> C[Decision Point]
    C --> D{{Yes or No?}}
    D -->|Yes| E[Action 1]
    D -->|No| F[Action 2]
    E --> G[Result 1]
    F --> H[Result 2]"""
        }
        
        # Check if we have a specific template for the topic
        for key, template in templates.items():
            if key in topic:
                return template
        
        # If no specific template, use default
        return templates["default"]
    
    def _generate_sequence_diagram(self, topic: str) -> str:
        """
        Generate a sequence diagram for the given topic
        """
        templates = {
            "ros communication": """sequenceDiagram
    participant N as Node A
    participant R as ROS 2 Middleware
    participant M as Node B
    
    N->>R: Publish Message
    R->>M: Deliver Message
    M->>R: Acknowledge Receipt
    R->>N: Status Update""",
            "service call": """sequenceDiagram
    participant C as Client Node
    participant S as Service Server
    
    C->>S: Request Service
    S->>C: Process Request
    C->>S: Get Response
    S->>C: Return Result""",
            "default": f"""sequenceDiagram
    participant A as Component A
    participant B as Component B
    
    A->>B: Send Request
    B->>A: Process & Respond"""
        }
        
        for key, template in templates.items():
            if key in topic:
                return template
        
        return templates["default"]
    
    def _generate_class_diagram(self, topic: str) -> str:
        """
        Generate a class diagram for the given topic
        """
        templates = {
            "robot class": """classDiagram
    class Robot {
        +String id
        +float[] position
        +float[] orientation
        +sensor_data: SensorData[]
        +move(target: float[])
        +sense(): SensorData
        +communicate(msg: string)
    }
    
    class Sensor {
        +String type
        +float range
        +int resolution
        +read(): float[]
    }
    
    class Actuator {
        +String type
        +float limits
        +apply_force(force: float[])
    }
    
    Robot --> Sensor : has
    Robot --> Actuator : has""",
            "ros node": """classDiagram
    class Node {
        +String node_name
        +Publisher[] publishers
        +Subscriber[] subscribers
        +Timer[] timers
        +init()
        +spin()
        +destroy()
    }
    
    class Publisher~T~ {
        +String topic_name
        +publish(msg: T)
    }
    
    class Subscriber~T~ {
        +String topic_name
        +callback_function
        +handle_message(msg: T)
    }
    
    Node --> Publisher : contains
    Node --> Subscriber : contains""",
            "default": f"""classDiagram
    class {topic.replace(' ', '')} {{
        +attribute1
        +attribute2
        +method1()
        +method2()
    }}"""
        }
        
        for key, template in templates.items():
            if key in topic:
                return template
        
        return templates["default"]
    
    def _generate_state_diagram(self, topic: str) -> str:
        """
        Generate a state diagram for the given topic
        """
        templates = {
            "robot state": """stateDiagram-v2
    [*] --> Idle
    Idle --> Moving: Command Received
    Moving --> Idle: Goal Reached
    Idle --> ObstacleDetected: Obstacle Ahead
    ObstacleDetected --> Moving: Path Cleared
    ObstacleDetected --> Idle: Stop Command
    Moving --> EmergencyStop: Critical Error
    EmergencyStop --> [*]: Reset""",
            "navigation": """stateDiagram-v2
    [*] --> Localized
    Localized --> Planning: Goal Set
    Planning --> Executing: Path Planned
    Executing --> Localized: Goal Reached
    Executing --> Replanning: Path Blocked
    Replanning --> Executing: New Path
    Executing --> [*]: Abort""",
            "default": f"""stateDiagram-v2
    [*] --> State1
    State1 --> State2: Event1
    State2 --> State1: Event2
    State2 --> [*]: End"""
        }
        
        for key, template in templates.items():
            if key in topic:
                return template
        
        return templates["default"]
    
    def _generate_er_diagram(self, topic: str) -> str:
        """
        Generate an entity-relationship diagram
        """
        return f"""erDiagram
    {topic.replace(' ', '').upper()} ||--o{{ RELATION : has
    {topic.replace(' ', '').upper()} {{
        string id
        string name
        date created_at
    }}
    RELATION {{
        string id
        string {topic.replace(' ', '').lower()}_id
        string related_entity_id
    }}"""
    
    def _generate_user_journey(self, topic: str) -> str:
        """
        Generate a user journey diagram
        """
        return f"""journey
    title User Journey for {topic}
    section Understanding
      Learn about {topic}: 3: User
    section Implementation
      Apply {topic}: 4: User
    section Application
      Use {topic} Practically: 5: User"""
    
    def _generate_gantt_chart(self, topic: str) -> str:
        """
        Generate a Gantt chart
        """
        return f"""gantt
    title {topic} Development Timeline
    dateFormat  YYYY-MM-DD
    section {topic} Planning
    Research          :done, des1, 2023-01-01, 30d
    Design            :active,  des2, 2023-01-31, 40d
    Implementation    :         des3, after des2, 60d
    Testing           :         des4, after des3, 30d"""
    
    def _generate_pie_chart(self, topic: str) -> str:
        """
        Generate a pie chart
        """
        return f"""pie title {topic} Distribution
    "Category A" : 40
    "Category B" : 30
    "Category C" : 20
    "Category D" : 10"""
    
    def _determine_usage_context(self, topic: str) -> str:
        """
        Determine where the diagram would be used in the textbook
        """
        if "ros" in topic.lower() or "node" in topic.lower():
            return "ROS 2 module - architecture and communication"
        elif "navigation" in topic.lower() or "planning" in topic.lower():
            return "Path planning and navigation chapters"
        elif "control" in topic.lower():
            return "Robot control systems chapters"
        elif "perception" in topic.lower() or "sensor" in topic.lower():
            return "Perception and sensing chapters"
        else:
            return "General robotics concepts chapters"

def generate_diagram(topic: str, diagram_type: str = "flowchart", description: str = None) -> Dict[str, Any]:
    """
    Main function to generate diagram
    This would be called by the Qwen CLI
    """
    generator = DiagramGeneratorSkill()
    result = generator.generate_diagram(topic, diagram_type, description)
    return result

if __name__ == "__main__":
    print("Diagram Generator Skill initialized")
    
    # Example usage
    result = generate_diagram(
        topic="ROS Node Communication",
        diagram_type="sequence"
    )
    print(f"Generated diagram for: {result['topic']}")
    print(f"Diagram type: {result['diagram_type']}")
    print(f"Usage context: {result['usage_context']}")