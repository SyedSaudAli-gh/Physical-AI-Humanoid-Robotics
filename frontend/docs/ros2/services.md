---
sidebar_position: 4
---

# ROS 2 Services

Services in ROS 2 provide a request-response communication pattern that complements the publish-subscribe model of topics. Unlike topics, services are synchronous and request-response based, making them ideal for operations that require a specific response to a request.

## Learning Outcomes

By the end of this chapter, you will be able to:
- Understand the request-response communication model in ROS 2
- Create custom service definitions for your robotic applications
- Implement service servers and clients
- Choose between topics and services for different communication needs

## Theoretical Foundations

Services in ROS 2 implement a synchronous request-response communication pattern where a client sends a request to a server and waits for a response. This is different from topics, which are asynchronous and one-to-many by default.

Services are defined by a .srv file that specifies the request and response message types. The communication is bidirectional, with a clear request and response structure.

### Key Concepts

1. **Service Types**: Services are defined by .srv files containing both request and response message structures.

2. **Synchronous Communication**: The client waits for the server to process the request and return a response.

3. **Reliability**: Services guarantee delivery and processing of requests, making them suitable for critical operations.

## Practical Simulations

Let's look at how to define and implement services in ROS 2.

### Creating a Custom Service

First, let's define a custom service for robot navigation:

```
# NavigateToPose.srv
# Request
geometry_msgs/Pose target_pose
string navigation_mode

---
# Response
bool success
string message
float64 execution_time
```

### Python Service Server Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    
    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Service Client Example

```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    minimal_client.get_logger().info(
        f'Result of {sys.argv[1]} + {sys.argv[2]} = {response.sum}'
    )
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Examples

Here's a complete example of a service for controlling robot movement:

### Movement Control Service

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class MovementControlService(Node):
    def __init__(self):
        super().__init__('movement_control_service')
        self.srv = self.create_service(
            Trigger, 
            'start_movement', 
            self.handle_start_movement
        )
        self.movement_active = False

    def handle_start_movement(self, request, response):
        if not self.movement_active:
            self.get_logger().info('Starting robot movement...')
            # Simulate movement logic here
            self.movement_active = True
            response.success = True
            response.message = "Movement started successfully"
        else:
            response.success = False
            response.message = "Movement already active"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    service = MovementControlService()
    
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## When to Use Services vs. Topics

Services are appropriate for:
- Operations that require a specific response
- Actions that should be acknowledged (e.g., "move to position" with success/failure)
- Synchronous operations where the caller needs to wait for completion
- Commands that change robot state

Topics are better for:
- Continuous data streams (sensor readings, robot state)
- One-to-many broadcasting
- Asynchronous communication
- High-frequency updates

## Chapter Summary

This chapter explored ROS 2 services, which provide synchronous request-response communication between nodes. We covered how to define custom services, implement service servers and clients, and when to use services versus topics. Understanding both communication patterns is crucial for designing well-structured robotic applications.